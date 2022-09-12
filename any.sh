#!/bin/bash


if [[ -f "$IMAGE" ]]; then
	rm AnyKernel3/*.zip > /dev/null 2>&1
	cp $IMAGE AnyKernel3/Image.gz
	cd AnyKernel3
	zip -r9 Kranel-r8q.zip .
fi
