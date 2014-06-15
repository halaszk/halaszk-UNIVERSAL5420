subdir-ccflags-y := -marm -mfpu=neon-vfpv4 -ftree-vectorize -mvectorize-with-neon-quad -ffast-math -fmodulo-sched -fmodulo-sched-allow-regmoves -floop-interchange -ftree-loop-distribution -floop-strip-mine -floop-block -fgraphite-identity -fgcse-las -fgcse-sm -fivopts -ftree-loop-im -ftree-loop-ivcanon -fweb -ftracer -fsched-spec-load -fforce-addr -fsingle-precision-constant -funroll-loops -fomit-frame-pointer -fsection-anchors -fstrict-aliasing -Wstrict-aliasing=3
subdir-ldflags-y += -flto=4 -fuse-linker-plugin --gc-sections
obj-y			+= drm/ vga/ stub/ ion/ arm/
obj-y			+= pvr/
