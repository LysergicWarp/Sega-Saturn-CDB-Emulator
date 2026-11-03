# Builds for the host what the harness loads and runs: libcdblock.so with GCC 14 for Linux and
# cdblock.dll with llvm-mingw for Windows x64 into build/ of the repository mounted at /src, and
# GNU binutils' as, ld and objcopy for sh-elf, which assemble card code, into build/binutils/linux/
# and windows/. The two platforms' binutils sit apart, as Windows runs an extensionless file before
# the .exe beside it. The libraries link the C++ runtime statically, so everything needs only the C
# library. Each library builds in its own copy of the sources, so neither's objects reach build/,
# where a native make keeps its own.
#
#   docker build -t cdblock-build .
#   docker run --rm -v "$PWD:/src" --user "$(id -u):$(id -g)" cdblock-build   # Linux
#   docker run --rm -v "${PWD}:/src" cdblock-build                            # Windows, PowerShell

FROM gcc:14@sha256:1ea81e094f614fd2ed066316651dbac8eecb4d36add2ddd8a26151374c85c52c

ARG LLVM_MINGW=llvm-mingw-20261020-ucrt-ubuntu-22.04-x86_64
ARG LLVM_MINGW_SHA256=55064b60b991a1e1e8090b8c2f84e82b22cfe5903d12c159d65064cf5e8ae335
RUN curl -fsSL -o /tmp/llvm-mingw.tar.xz \
        https://github.com/mstorsjo/llvm-mingw/releases/download/20261020/$LLVM_MINGW.tar.xz \
    && echo "$LLVM_MINGW_SHA256  /tmp/llvm-mingw.tar.xz" | sha256sum -c - \
    && mkdir /opt/llvm-mingw \
    && tar -xJf /tmp/llvm-mingw.tar.xz -C /opt/llvm-mingw --strip-components=1 \
    && rm /tmp/llvm-mingw.tar.xz
ENV PATH=/opt/llvm-mingw/bin:$PATH

# The tarball's GNU signature was checked when it was pinned.
ARG BINUTILS=binutils-2.47
ARG BINUTILS_SHA256=154ab23b60070e8f27013c22977f1129425d67d1e8acd6e13010e617811e4cff
RUN <<"EOF"
set -e
curl -fsSL -o /tmp/binutils.tar.xz https://ftp.gnu.org/gnu/binutils/$BINUTILS.tar.xz
echo "$BINUTILS_SHA256  /tmp/binutils.tar.xz" | sha256sum -c -
tar -xJf /tmp/binutils.tar.xz -C /tmp
for platform in linux windows; do
    mkdir -p /tmp/build-$platform /opt/binutils/$platform
    cd /tmp/build-$platform
    if [ $platform = windows ]; then host="--host=x86_64-w64-mingw32"; exe=.exe; else host=; exe=; fi
    /tmp/$BINUTILS/configure --target=sh-elf $host --prefix=/tmp/install-$platform --disable-nls \
        --disable-werror --disable-plugins --disable-gprofng --without-zstd --without-debuginfod \
        --without-msgpack LDFLAGS=-s
    # Installed, not taken from the build tree, where a Windows host's programs are libtool wrappers.
    make -j"$(nproc)" all-gas all-ld all-binutils
    make install-gas install-ld install-binutils
    for tool in as ld objcopy; do
        cp /tmp/install-$platform/bin/sh-elf-$tool$exe /opt/binutils/$platform/
    done
done
rm -rf /tmp/*
EOF

COPY --chmod=755 <<"EOF" /usr/local/bin/build-cdblock
#!/bin/sh
set -e
for platform in linux windows; do
    rm -rf /tmp/$platform
    mkdir /tmp/$platform
    cp -r Makefile include src /tmp/$platform
done
make -C /tmp/linux -j"$(nproc)" build/libcdblock.so LDFLAGS="-static-libstdc++ -static-libgcc"
make -C /tmp/windows -j"$(nproc)" build/cdblock.dll OS=Windows_NT CXX=x86_64-w64-mingw32-clang++
mkdir -p build/binutils
cp /tmp/linux/build/libcdblock.so /tmp/windows/build/cdblock.dll build/
cp -r /opt/binutils/linux /opt/binutils/windows build/binutils/
EOF

WORKDIR /src
CMD ["build-cdblock"]
