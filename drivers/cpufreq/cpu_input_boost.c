
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

#define pr_fmt(fmt) "cpu_input_boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/msm_drm_notify.h>
#include <linux/slab.h>
#include <linux/version.h>

/* The sched_param struct is located elsewhere in newer kernels */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
#include <uapi/linux/sched/types.h>
#endif

enum {
	SCREEN_OFF,
	INPUT_BOOST,
	MAX_BOOST
};

struct boost_drv {
	struct delayed_work input_unboost;
	struct delayed_work max_unboost;
	struct notifier_block cpu_notif;
	struct notifier_block msm_drm_notif;
	wait_queue_head_t boost_waitq;
	atomic_long_t max_boost_expires;
	unsigned long state;
};

static void input_unboost_worker(struct work_struct *work);
static void max_unboost_worker(struct work_struct *work);

static struct boost_drv boost_drv_g __read_mostly = {
	.input_unboost = __DELAYED_WORK_INITIALIZER(boost_drv_g.input_unboost,
						    input_unboost_worker, 0),
	.max_unboost = __DELAYED_WORK_INITIALIZER(boost_drv_g.max_unboost,
						  max_unboost_worker, 0),
	.boost_waitq = __WAIT_QUEUE_HEAD_INITIALIZER(boost_drv_g.boost_waitq)
};

static unsigned int get_input_boost_freq(struct cpufreq_policy *policy)
{
	unsigned int freq;

	if (cpumask_test_cpu(policy->cpu, cpu_lp_mask))
		freq = CONFIG_INPUT_BOOST_FREQ_LP;
	else if (cpumask_test_cpu(policy->cpu, cpu_perf_mask))
		freq = CONFIG_INPUT_BOOST_FREQ_PERF;
	else
		freq = CONFIG_INPUT_BOOST_FREQ_PERFP;
	return min(freq, policy->max);
}

static unsigned int get_max_boost_freq(struct cpufreq_policy *policy)
{
	unsigned int freq;

	if (cpumask_test_cpu(policy->cpu, cpu_lp_mask))
		freq = CONFIG_MAX_BOOST_FREQ_LP;
	else if (cpumask_test_cpu(policy->cpu, cpu_perf_mask))
		freq = CONFIG_MAX_BOOST_FREQ_PERF;
	else
		freq = CONFIG_MAX_BOOST_FREQ_PERFP;
	return min(freq, policy->max);
}

static void update_online_cpu_policy(void)
{
	unsigned int cpu;

	/* Only one CPU from each cluster needs to be updated */
	get_online_cpus();
	cpu = cpumask_first_and(cpu_lp_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	cpu = cpumask_first_and(cpu_perf_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	cpu = cpumask_first_and(cpu_prime_mask, cpu_online_mask);
	cpufreq_update_policy(cpu);
	put_online_cpus();
}

static void __cpu_input_boost_kick(struct boost_drv *b)
{
	if (test_bit(SCREEN_OFF, &b->state))
		return;

	set_bit(INPUT_BOOST, &b->state);
	if (!mod_delayed_work(system_unbound_wq, &b->input_unboost,
			      msecs_to_jiffies(CONFIG_INPUT_BOOST_DURATION_MS)))
		wake_up(&b->boost_waitq);
}

void cpu_input_boost_kick(void)
{
	struct boost_drv *b = &boost_drv_g;

	__cpu_input_boost_kick(b);
}

static void __cpu_input_boost_kick_max(struct boost_drv *b,
				       unsigned int duration_ms)
{
	unsigned long boost_jiffies = msecs_to_jiffies(duration_ms);
	unsigned long curr_expires, new_expires;

	if (test_bit(SCREEN_OFF, &b->state))
		return;

	do {
		curr_expires = atomic_long_read(&b->max_boost_expires);
		new_expires = jiffies + boost_jiffies;

		/* Skip this boost if there's a longer boost in effect */
		if (time_after(curr_expires, new_expires))
			return;
	} while (atomic_long_cmpxchg(&b->max_boost_expires, curr_expires,
				     new_expires) != curr_expires);

	set_bit(MAX_BOOST, &b->state);
	if (!mod_delayed_work(system_unbound_wq, &b->max_unboost,
			      boost_jiffies))
		wake_up(&b->boost_waitq);
}

void cpu_input_boost_kick_max(unsigned int duration_ms)
{
	struct boost_drv *b = &boost_drv_g;

	__cpu_input_boost_kick_max(b, duration_ms);
}

static void input_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					   typeof(*b), input_unboost);

	clear_bit(INPUT_BOOST, &b->state);
	wake_up(&b->boost_waitq);
}

static void max_unboost_worker(struct work_struct *work)
{
	struct boost_drv *b = container_of(to_delayed_work(work),
					   typeof(*b), max_unboost);

	clear_bit(MAX_BOOST, &b->state);
	wake_up(&b->boost_waitq);
}

static int cpu_boost_thread(void *data)
{
	static const struct sched_param sched_max_rt_prio = {
		.sched_priority = MAX_RT_PRIO - 1
	};
	struct boost_drv *b = data;
	unsigned long old_state = 0;

	sched_setscheduler_nocheck(current, SCHED_FIFO, &sched_max_rt_prio);

	while (1) {
		bool should_stop = false;
		unsigned long curr_state;

		wait_event(b->boost_waitq,
			(curr_state = READ_ONCE(b->state)) != old_state ||
			(should_stop = kthread_should_stop()));

		if (should_stop)
			break;

		old_state = curr_state;
		update_online_cpu_policy();
	}

	return 0;
}

static int cpu_notifier_cb(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), cpu_notif);
	struct cpufreq_policy *policy = data;

	if (action != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	/* Unboost when the screen is off */
	if (test_bit(SCREEN_OFF, &b->state)) {
		policy->min = policy->cpuinfo.min_freq;
		return NOTIFY_OK;
	}

	/* Boost CPU to max frequency for max boost */
	if (test_bit(MAX_BOOST, &b->state)) {
		policy->min = get_max_boost_freq(policy);
		return NOTIFY_OK;
	}

	/*
	 * Boost to policy->max if the boost frequency is higher. When
	 * unboosting, set policy->min to the absolute min freq for the CPU.
	 */
	if (test_bit(INPUT_BOOST, &b->state))
		policy->min = get_input_boost_freq(policy);
	else
		policy->min = policy->cpuinfo.min_freq;

	return NOTIFY_OK;
}

static int msm_drm_notifier_cb(struct notifier_block *nb, unsigned long action,
			  void *data)
{
	struct boost_drv *b = container_of(nb, typeof(*b), msm_drm_notif);
	struct msm_drm_notifier *evdata = data;
	int *blank = evdata->data;

	/* Parse framebuffer blank events as soon as they occur */
	if (action != MSM_DRM_EARLY_EVENT_BLANK)
		return NOTIFY_OK;

	/* Boost when the screen turns on and unboost when it turns off */
	if (*blank == MSM_DRM_BLANK_UNBLANK_CUST) {
		clear_bit(SCREEN_OFF, &b->state);
		__cpu_input_boost_kick_max(b, CONFIG_WAKE_BOOST_DURATION_MS);
	} else {
		set_bit(SCREEN_OFF, &b->state);
		wake_up(&b->boost_waitq);
	}

	return NOTIFY_OK;
}

static void cpu_input_boost_input_event(struct input_handle *handle,
					unsigned int type, unsigned int code,
					int value)
{
	struct boost_drv *b = handle->handler->private;

	__cpu_input_boost_kick(b);
}

static int cpu_input_boost_input_connect(struct input_handler *handler,
					 struct input_dev *dev,
					 const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);

/*
 * Copyright (C) 2014-2015, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "CPU-boost: " fmt

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/slab.h>

enum boost_status {
	UNBOOST,
	BOOST,
};

enum boost_pwr {
	LOW,
	MID,
	HIGH,
};

struct boost_policy {
	enum boost_status boost_state;
};

static DEFINE_PER_CPU(struct boost_policy, boost_info);
static struct workqueue_struct *boost_wq;
static struct work_struct boost_work;
static struct delayed_work restore_work;

static bool boost_running;

static u64 last_input_time;
#define MIN_INPUT_INTERVAL (150 * USEC_PER_MSEC)

/**
 * Auto boost freq calculation:
 * Requested boost freqs = maxfreq * boost_factor[i] / BOOST_FACTOR_DIVISOR,
 * so the lowest boost freq in this case would be maxfreq * 3 / 7
 */
static unsigned int boost_freq[3];
static unsigned int boost_factor[3] = {3, 4, 5};
#define BOOST_FACTOR_DIVISOR 7

/* Boost-freq level to use (high, mid, low) */
static enum boost_pwr boost_level;

/* Boost duration in millsecs */
static unsigned int boost_ms;

/* On/off switch */
static unsigned int enabled;
module_param(enabled, uint, 0644);

/**
 * Percentage threshold used to boost CPUs (default 30%). A higher
 * value will cause more CPUs to be boosted -- CPUs are boosted
 * when ((current_freq/max_freq) * 100) < up_threshold
 */
static unsigned int up_threshold = 30;
module_param(up_threshold, uint, 0644);

static void cpu_unboost_all(void)
{
	struct boost_policy *b;
	unsigned int cpu;

	get_online_cpus();
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (b->boost_state == BOOST) {
			b->boost_state = UNBOOST;
			if (cpu_online(cpu))
				cpufreq_update_policy(cpu);
		}
	}
	put_online_cpus();
	boost_running = false;
}

static void __cpuinit cpu_boost_main(struct work_struct *work)
{
	struct boost_policy *b;
	struct cpufreq_policy *policy;
	unsigned int cpu, num_cpus_boosted = 0, num_cpus_to_boost = 0;

	/* Num of CPUs to be boosted based on current freq of each online CPU */
	get_online_cpus();
	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (policy != NULL) {
			if ((policy->cur * 100 / policy->max) < up_threshold)
				num_cpus_to_boost++;
			cpufreq_cpu_put(policy);
			/* Only allow 2 CPUs to be staged for boosting from here */
			if (num_cpus_to_boost == 2)
				break;
		}
	}

	/* Num of CPUs to be boosted based on how many of them are online */
	switch (num_online_cpus() * 100 / CONFIG_NR_CPUS) {
	case 25:
		num_cpus_to_boost += 2;
		break;
	case 50 ... 75:
		num_cpus_to_boost++;
		break;
	}

	/* Nothing to boost */
	if (!num_cpus_to_boost) {
		put_online_cpus();
		boost_running = false;
		return;
	}

	/* Boost freq to use based on how many CPUs to boost */
	switch (num_cpus_to_boost * 100 / CONFIG_NR_CPUS) {
	case 25:
		boost_level = HIGH;
		break;
	case 50:
		boost_level = MID;
		break;
	default:
		boost_level = LOW;
	}

	/* Dual-core systems need more power */
	if (CONFIG_NR_CPUS == 2)
		boost_level++;

	/* Calculate boost duration */
	boost_ms = 3000 - ((num_cpus_to_boost * 750) + ((boost_level + 1) * 250));

	/* Prioritize boosting of online CPUs */
	for_each_online_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		b->boost_state = BOOST;
		cpufreq_update_policy(cpu);
		num_cpus_boosted++;
		if (num_cpus_boosted == num_cpus_to_boost)
			goto finish_boost;
	}

	/* Boost offline CPUs if we still need to boost more CPUs */
	for_each_possible_cpu(cpu) {
		b = &per_cpu(boost_info, cpu);
		if (b->boost_state == UNBOOST) {
			b->boost_state = BOOST;
			num_cpus_boosted++;
			if (num_cpus_boosted == num_cpus_to_boost)
				goto finish_boost;
		}
	}

finish_boost:
	put_online_cpus();
	queue_delayed_work(boost_wq, &restore_work,
				msecs_to_jiffies(boost_ms));
}

static void __cpuinit cpu_restore_main(struct work_struct *work)
{
	cpu_unboost_all();
}

static int cpu_do_boost(struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct boost_policy *b = &per_cpu(boost_info, policy->cpu);

	if (val != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	switch (b->boost_state) {
	case UNBOOST:
		policy->min = policy->cpuinfo.min_freq;
		break;
	case BOOST:
		if (boost_freq[boost_level] > policy->max)
			policy->min = policy->max;
		else
			policy->min = boost_freq[boost_level];
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu_do_boost_nb = {
	.notifier_call = cpu_do_boost,
};

static void cpu_boost_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	u64 now;

	if (boost_running)
		return;
	if (!enabled)
		return;

	now = ktime_to_us(ktime_get());
	if (now - last_input_time < MIN_INPUT_INTERVAL)
		return;

	boost_running = true;
	queue_work(boost_wq, &boost_work);
	last_input_time = ktime_to_us(ktime_get());
}

static int cpu_boost_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);

	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;

	handle->name = "cpu_input_boost_handle";

	ret = input_register_handle(handle);
	if (ret)
		goto free_handle;

	ret = input_open_device(handle);
	if (ret)
		goto unregister_handle;

	return 0;

unregister_handle:
	input_unregister_handle(handle);
free_handle:
	kfree(handle);
	return ret;
}

static void cpu_input_boost_input_disconnect(struct input_handle *handle)

	handle->name = "cpu_input_boost";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void cpu_boost_input_disconnect(struct input_handle *handle)

{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id cpu_input_boost_ids[] = {
	/* Multi-touch touchscreen */

static const struct input_device_id cpu_boost_ids[] = {
	/* multi-touch touchscreen */

	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |

			BIT_MASK(ABS_MT_POSITION_Y) }
	},
	/* Touchpad */

			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */

	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =

			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) }
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) }
	},
	{ }
};

static struct input_handler cpu_input_boost_input_handler = {
	.event		= cpu_input_boost_input_event,
	.connect	= cpu_input_boost_input_connect,
	.disconnect	= cpu_input_boost_input_disconnect,
	.name		= "cpu_input_boost_handler",
	.id_table	= cpu_input_boost_ids

			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	{ },
};

static struct input_handler cpu_boost_input_handler = {
	.event		= cpu_boost_input_event,
	.connect	= cpu_boost_input_connect,
	.disconnect	= cpu_boost_input_disconnect,
	.name		= "cpu_input_boost",
	.id_table	= cpu_boost_ids,

};

static int __init cpu_input_boost_init(void)
{

	struct boost_drv *b = &boost_drv_g;
	struct task_struct *thread;
	int ret;

	b->cpu_notif.notifier_call = cpu_notifier_cb;
	ret = cpufreq_register_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		pr_err("Failed to register cpufreq notifier, err: %d\n", ret);
		return ret;
	}

	cpu_input_boost_input_handler.private = b;
	ret = input_register_handler(&cpu_input_boost_input_handler);
	if (ret) {
		pr_err("Failed to register input handler, err: %d\n", ret);
		goto unregister_cpu_notif;
	}

	b->msm_drm_notif.notifier_call = msm_drm_notifier_cb;
	b->msm_drm_notif.priority = INT_MAX;
	ret = msm_drm_register_client(&b->msm_drm_notif);
	if (ret) {
		pr_err("Failed to register msm_drm notifier, err: %d\n", ret);
		goto unregister_handler;
	}

	thread = kthread_run(cpu_boost_thread, b, "cpu_boostd");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		pr_err("Failed to start CPU boost thread, err: %d\n", ret);
		goto unregister_fb_notif;
	}

	return 0;

unregister_fb_notif:
	msm_drm_unregister_client(&b->msm_drm_notif);
unregister_handler:
	input_unregister_handler(&cpu_input_boost_input_handler);
unregister_cpu_notif:
	cpufreq_unregister_notifier(&b->cpu_notif, CPUFREQ_POLICY_NOTIFIER);
	return ret;
}
subsys_initcall(cpu_input_boost_init);

	struct cpufreq_frequency_table *table = cpufreq_frequency_get_table(0);
	int maxfreq = cpufreq_quick_get_max(0);
	int b_level = 0, req_freq[3];
	int i, ret = 1;

	if (!maxfreq) {
		pr_err("Failed to get max freq, input boost disabled\n");
		goto err;
	}

	/* Calculate ideal boost freqs */
	for (i = 0; i < 3; i++)
		req_freq[i] = maxfreq * boost_factor[i] / BOOST_FACTOR_DIVISOR;

	/* Find actual freqs closest to ideal boost freqs */
	for (i = 0;; i++) {
		int curr = table[i].frequency - req_freq[b_level];
		int prev = table[i ? i - 1 : 0].frequency - req_freq[b_level];

		if (!curr || (curr > 0 && prev < 0)) {
			boost_freq[b_level] = table[i].frequency;
			b_level++;
			if (b_level == 3)
				break;
		}
	}

	boost_wq = alloc_workqueue("cpu_input_boost_wq", WQ_HIGHPRI | WQ_NON_REENTRANT, 0);
	if (!boost_wq) {
		pr_err("Failed to allocate workqueue\n");
		ret = -EFAULT;
		goto err;
	}

	cpufreq_register_notifier(&cpu_do_boost_nb, CPUFREQ_POLICY_NOTIFIER);

	INIT_DELAYED_WORK(&restore_work, cpu_restore_main);
	INIT_WORK(&boost_work, cpu_boost_main);

	ret = input_register_handler(&cpu_boost_input_handler);
	if (ret)
		pr_err("Failed to register input handler, err: %d\n", ret);
err:
	return ret;
}
late_initcall(cpu_input_boost_init);

MODULE_AUTHOR("Sultanxda <sultanxda@gmail.com>");
MODULE_DESCRIPTION("CPU Input Boost");
MODULE_LICENSE("GPLv2");

