# SCons version problem:
# 	python3 /Tools/scons/script/scons.py <options>
#	or
# 	Tools/scons$ python3 script/scons.py -C <dir> <options>
#
# SCONS_CMD=scons
# for Gem5 ver. >= 21
#SCONS_CMD=python3 /Tools/scons/scripts/scons.py
SCONS_CMD=scons

N_CPU=4
M5_PATH=/Data/Gem5/arm
# M5_PATH=/Data/Gem5/aarch-system-201700616
# [    0.283288] Kernel panic - not syncing: VFS: Unable to mount root fs on unknown-block(0,0)
# M5_PATH=/Data/Gem5/aarch-system-201901106
# M5_PATH=/Data/Gem5/aarch-system-20210904
#

all:
	 @echo "Build: X86 ARM MIPS POWER"
	 @echo "	Requires python3 >= 3.5 (recommend 3.7)"
	 @echo "	Do export M5_PATH=$(M5_PATH) first for ARM full system"
	 @echo "Part1: p1s p1t"
	 @echo "Part2: p2h p2s p2c p2m"
	 @echo "Part3: p3mc p3rc p3rt p3sr p3ts"
	 @echo "examples: armwj fs arm_bl term"
	 @echo "	xfs	X86 Linux Full System"
	 @echo "	armwj	ARM Linux Full System"
	 @echo "	fs_bl	X86 Full System for ARM Big-Littel Architecture"
	 @echo "	term	Connect dummy terminal to Full System"
	 @echo "	xmn	MobileNet v2"
	 @echo "ARM Research Starter Kit:"
	 @echo "	armse	ARM Starter SE Mode"
	 @echo "	armse2	ARM Starter SE Mode with 2 cores"
	 @echo "	armfs	ARM Linux Full System"
	 @echo "	arms	ARM Starter Kit"
	 @echo "	hpise	HPI Model SE Mode"
	 @echo "	hpise2	HPI Model SE Mode with 2 cores"
	 @echo "	hpisebm	HPI Model SE Mode Benchmarks"
	 @echo "	hpifs	HPI Model FS Mode"
	 @echo "	amn	MobileNet v2 with ARM HPI"
	 @echo "Memory System"
	 @echo "	MI	MI example"
	 @echo "nw: Network Interface Simulation"
	 @echo "pu: PU DMA ENgine"

include Makefile.nw

#
# Pu DMA model development
#
PUTEST = putest
PUTEST_X86 = ./tests/pudma/bin/x86/linux/$(PUTEST)32-static
PUTEST_ARM = ./tests/pudma/bin/arm/linux/$(PUTEST)32-static
PUTEST_SRC = ./tests/pudma/src

M5OUT_PUTEST = ./m5out/pudma
 
$(PUTEST_X86) : $(PUTEST_SRC)/$(PUTEST).c
	cd $(PUTEST_SRC); make -f Makefile.x86

pumem3: $(PUTEST_X86)
	./build/X86/gem5.opt \
	   	--outdir=$(M5OUT_PUTEST)/pumem3 \
		--debug-flags=PuEngine2,PuEngine3 \
		configs/pudma/pumem3.py
	@echo "See $(M5OUT_PUTEST)/pumem3 directory\n\n"

pumem2: $(PUTEST_X86)
	./build/X86/gem5.opt \
	   	--outdir=$(M5OUT_PUTEST)/pumem2\
		--debug-flags=PuEngine2 \
		configs/pudma/pumem2.py
	@echo "See $(M5OUT_PUTEST)/pumem2 directory\n\n"

pumem: $(PUTEST_X86)
	./build/X86/gem5.opt \
	   	--outdir=$(M5OUT_PUTEST)/pumem \
		--debug-flags=PUDMA \
		configs/pudma/pumem.py
	@echo "See $(M5OUT_PUTEST)/pumem directory\n\n"

git-soarstill:
pu: $(PUTEST_X86)
	./build/X86/gem5.opt \
	   	--outdir=$(M5OUT_PUTEST)/puse \
		--debug-flags=PUDMA \
		configs/pudma/puse.py --cmd=$(PUTEST_X86)
	@echo "See $(M5OUT_PUTEST)/puse directory\n\n"


pusimple: $(PUTEST_X86)
	./build/X86/gem5.opt \
	   	--outdir=$(M5OUT_PUTEST)/pusimple \
		--debug-flags=PUDMA \
		configs/pudma/pusimple.py
	@echo "See $(M5OUT_PUTEST)/pusimple directory\n\n"

git-soarstill:
	git config --global user.name soarstill
	git config --global user.email soarstill@gmail
	git config --global credential.helper cache
	git config --global push.default simple




#
# Build Commands
#
X86: 
	$(SCONS_CMD) build/X86/gem5.opt -j8

ARM:
	$(SCONS_CMD) build/ARM/gem5.opt -j8

MIPS:
	$(SCONS_CMD) build/MIPS/gem5.opt -j8

POWER:
	$(SCONS_CMD) build/POWER/gem5.opt -j8

# Part 1
# Simple Architecture
p1s:
	./build/X86/gem5.opt configs/learning_gem5/part1/simple.py
	echo "See m5out directory"

# Two-level Cache Architecture
p1t:
	./build/X86/gem5.opt configs/learning_gem5/part1/two_level.py
#	./build/X86/gem5.opt wjkim/learning_gem5/part1/two_level.py
	echo "See m5out directory"

# Part 2
p2h:
	./build/X86/gem5.opt configs/learning_gem5/part2/putest_goodbye.py

p2s:
	./build/X86/gem5.opt configs/learning_gem5/part2/run_simple.py

p2c:
	./build/X86/gem5.opt configs/learning_gem5/part2/simple_cache.py

p2m:
	./build/X86/gem5.opt configs/learning_gem5/part2/simple_memobj.py

# Part 3
p3mc:
	./build/X86/gem5.opt configs/learning_gem5/part3/msi_caches.py

p3rc:
	./build/X86/gem5.opt configs/learning_gem5/part3/ruby_caches_MI_example.py

p3rt:
	./build/X86/gem5.opt configs/learning_gem5/part3/ruby_test.py

p3sr:
	./build/X86/gem5.opt configs/learning_gem5/part3/simple_ruby.py

p3ts:
	./build/X86/gem5.opt configs/learning_gem5/part3/test_caches.py

armwj:
	build/ARM/gem5.opt configs/example/arm_fs.py --kernel /Data/Gem5/arm/binaries/vmlinux.arm64 --disk disks/arm/aarch64-ubuntu-trusty-headless.img --bootloader /Data/Gem5/arm/binaries/boot.arm64
#	build/ARM/gem5.opt configs/example/arm_fs.py --kernel /Data/Gem5/arm/binaries/vmlinux.arm64 --disk /Data/Gem5/arm/aarch64-ubuntu-trusty-headless.img --bootloader /Data/Gem5/arm/binaries/boot.arm64

xmn:
	time ./build/X86/gem5.opt configs/example/se.py \
		-c ../Mobilenet_CPP/mobilenet

#
# Examples
#
#	ARM Research Starter Kit
# System Call Emulation Mode
armse: tests/putest/bin/arm/linux/putest
	./build/ARM/gem5.opt configs/example/arm/starter_se.py --cpu="minor" \
		"tests/putest/bin/arm/linux/putest"

# System Call Emulation Multi-Core Mode
armse2: tests/putest/bin/arm/linux/putest
	time ./build/ARM/gem5.opt configs/example/arm/starter_se.py --cpu="minor" \
		--num-cores 2 \
		"tests/putest/bin/arm/linux/putest" \
		"tests/putest/bin/arm/linux/putest"

armse4: tests/putest/bin/arm/linux/putest
	time ./build/ARM/gem5.opt configs/example/arm/starter_se.py --cpu="minor" \
		--num-cores 4 \
		"tests/putest/bin/arm/linux/putest" \
		"tests/putest/bin/arm/linux/putest" \
		"tests/putest/bin/arm/linux/putest" \
		"tests/putest/bin/arm/linux/putest"

# HPI Model SE Mode
hpise: tests/putest/bin/arm/linux/putest
	time ./build/ARM/gem5.opt configs/example/arm/starter_se.py --cpu="hpi" --num-cores 1 \
		"tests/putest/bin/arm/linux/putest"

hpise2: tests/putest/bin/arm/linux/putest
	time ./build/ARM/gem5.opt configs/example/arm/starter_se.py --cpu="hpi" --num-cores 2 \
		"tests/putest/bin/arm/linux/putest" "tests/putest/bin/arm/linux/putest"

tests/putest/bin/arm/linux/putest:
	cd tests/putest/src; make -f Makefile.arm

BM='Quicksort'

hpisebm:
	./build/ARM/gem5.opt -d se_results/${BM} configs/example/arm/starter_se.py \
		--cpu="hpi" ../se-benchmarks/${BM}

# MobileNet v2
amn:
	time ./build/ARM/gem5.opt configs/example/arm/starter_se.py --cpu="hpi" \
		../Mobilenet_CPP/mobilenet_arm

# HPI Model FS Mode
hpifs:
	time ./build/ARM/gem5.opt configs/example/arm/starter_fs.py --cpu="hpi" --num-cores 1 \
		--disk-image=$(M5_PATH)/disks/linaro-minimal-aarch64.img


# ARM Full System (FS) Mode
CPUS=4
OUTDIR=m5out_afs
M5_PATH=/Data/Gem5/aarch-system-201901106

afs:
	export M5_PATH=/Data/Gem5/aarch-system-201901106
	./build/ARM/gem5.opt \
		--outdir=$(OUTDIR) \
		configs/example/fs.py \
		--cpu-clock=1GHz \
		--cpu-type="HPI" \
		-n $(CPUS) \
		--machine-type=VExpress_GEM5_V1 \
		--dtb-file=$(M5_PATH)/binaries/armv8_gem5_v1_$(CPUS)cpu.dtb \
		--kernel $(M5_PATH)/binaries/vmlinux.arm64 \
		--disk-image=$(M5_PATH)/disks/linaro-minimal-aarch64.img \
		--caches \
		--l2cache \
		--l1i_size=32kB \
		--l1d_size=32kB \
		--l2_size=1MB \
		--l2_assoc=2 \
		--mem-type=DDR4_2400_4x16 \
		--mem-ranks=4 \
		--mem-size=4GB \
		--sys-clock=1600MHz
	#	--disk-image=$(M5_PATH)/disks/ubuntu-18.04-arm64-docker.img \


armfs:
	export M5_PATH=$(M5_PATH)
	./build/ARM/gem5.opt \
		--outdir=m5out_armfs \
		configs/example/arm/starter_fs.py \
		--cpu "hpi" --num-cores $(N_CPU) \
		--disk-image=$(M5_PATH)/disks/linaro-minimal-aarch64.img \
		--dtb=$(M5_PATH)/binaries/armv7_gem5_v1_$(N_CPU)cpu.20170616.dtb \
		--kernel $(M5_PATH)/binaries/vmlinux.arm
	#	--bootloader $(M5_PATH)/binaries/boot.arm64
	#	--root=/dev/sda \
	#	--disk-image=$(M5_PATH)/disks/aarch64-ubuntu-trusty-headless.img \
	#	--disk-image $(M5_PATH)/ubuntu-18.04-arm64-docker.img
	#	--disk-image $(M5_PATH)/disks/m5_exit.squashfs.arm
	#	--disk-image=$(M5_PATH)/images/MobileNet/aarch64-ubuntu-trusty-headless.img
	#	--disk-image=$(M5_PATH)/disks/aarch64-ubuntu-trusty-headless.img

# ARM Full System with Mobilenet V2
armfsm:
	export M5_PATH=$(M5_PATH)
	./build/ARM/gem5.opt configs/example/arm/starter_fs.py --cpu "hpi" --num-cores ${N_CPU} \
		--kernel /Data/Gem5/arm/binaries/vmlinux.arm \
		--disk-image=$(M5_PATH)/images/MobileNet/aarch64-ubuntu-trusty-headless.img

armfs64:
	export M5_PATH=$(M5_PATH)
	./build/ARM/gem5.opt configs/example/arm/starter_fs.py --cpu "hpi" --num-cores 4 \
		--mem-size "8GB"
		--kernel $(M5_PATH)/binaries/vmlinux.arm64 \
		--disk-image=$(M5_PATH)/disks/aarch64-ubuntu-trusty-headless.img

# ARM Full System for Homomorphic Encryption
armhe:
	export M5_PATH=$(M5_PATH)
	./build/ARM/gem5.opt configs/example/arm/he_fs.py --cpu "hpi" --num-cores ${N_CPU} \
		--kernel $(M5_PATH)/binaries/vmlinux.arm64 \
		--disk-image=$(M5_PATH)/disks/aarch64-ubuntu-trusty-headless.img


# Full System Example  from Learning Gem5
# Downalod and extract file http://www.m5sim.org/dist/current/x86/x86-system.tar.bz2
#	Files are disks/linux-x86.img and binaries/x86_64-vmlinux-2.6.22.9  x86_64-vmlinux-2.6.22.9.smp

xfs:
	@echo "Execute in another terminal ./util/term/m5term localhost 3456"
	./build/X86/gem5.opt configs/example/fs.py

term: ./util/term/m5term
	./util/term/m5term localhost 3456

./util/term/m5term:
	cd ./util/term; make

# MI example
MI:
	@echo "ruby/MI example"
	./build/X86/gem5.opt configs/ruby/MI_example.py

gmesh:
	./build/ARM/gem5.opt \
		configs/example/ruby_random_test.py  \
		--num-cpus=16 \
		--num-dirs=16  \
		--network=garnet \
		--topology=Mesh_XY \
		--mesh-rows=4  

# Not complete yet
arm_bl:
	export M5_PATH=$(M5_PATH)
	./build/ARM/gem5.opt configs/example/arm/fs_bigLITTLE.py \
		--cpu-type=exynos --caches \
		--dtb=$(M5_PATH)/binaries/armv8_gem5_v1_big_little_2_2.dtb \
		--disk=$(M5_PATH)/disks/
		--kernel=$(M5_PATH)/binaries/vmlinux \
		--bootscript=$(M5_PATH)/binaries/boot.arm
		# --bm=home/nitesh/G/gem5/rodinia/rodinia/openmp/heartwall/heartwall

		--kernel $(M5_PATH)/binaries/vmlinux.arm64 --disk $(M5_PATH)/aarch64-ubuntu-trusty-headless.img --bootloader $(M5_PATH)/binaries/boot.arm64

mnt:
	sudo mount -o loop,offset=32256 disks/arm/aarch64-ubuntu-trusty-headless.img imgdir

umnt:
	sudo umount imgdir
