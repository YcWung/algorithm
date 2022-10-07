# Build

## Prerequisites

- conan
- cmake
- c++20 compiler. (win VS2022)

## Build

```bash
# build release version
./scripts/build.sh
# or ./scripts/build.sh -t Release

# build debug version
./scripts/build.sh -t Debug
```

## Test

```bash
# test release version
./scripts/test.sh
# or ./scripts/test.sh -t Release

# test debug version
./scripts/test.sh -t Debug
```
