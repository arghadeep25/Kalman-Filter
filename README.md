# Kalman Filter

![workflow](https://github.com/arghadeep25/Kalman-Filter/actions/workflows/release.yml/badge.svg)
[![Hits-of-Code](https://hitsofcode.com/github/arghadeep25/Kalman-Filter)](https://hitsofcode.com/github/arghadeep25/Data-Structures-and-Algorithms/view)
![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)
![C++](https://forthebadge.com/images/badges/made-with-c-plus-plus.svg)
![Data-Structures-and-Algorithms](https://socialify.git.ci/arghadeep25/Kalman-Filter/image?description=1&forks=1&language=1&name=1&owner=1&pattern=Charlie%20Brown&stargazers=1&theme=Dark)


## Content

| Content                                                                                  | Status  |
|------------------------------------------------------------------------------------------|---------|
| Kalman Filter                                                                            | &check; |
| Extended Kalman Filter                                                                   | &cross; |
| Unscented Kalman Filter                                                                                       | &cross; |




## Usage

```
git clone git@github.com:arghadeep25/Kalman-Filter.git
cd Kalman-Filter
```


#### Build Script (Recommended)
Build Helper
```
Usage: ./build.sh -b CMAKE_BUILD_TYPE -e BUILD_EXAMPLE
	-b CMAKE_BUILD_TYPE: Release | Debug
	-e BUILD_EXAMPLE: ON | OFF
```
```
./build.sh -b Release -e ON
```


#### Build Manually
```
mkdir build && cd build
cmake .. -DBUILD_EXAMPLE=ON -DCMAKE_BUILD_TYPE=Release
make -j4
```

Turning ON and OFF the examples might need to remove the build folder manually.

## Documentation
The documentation is done using Doxygen. If you don't have doxygen installed, use the following command.
```
sudo apt install doxygen
```

To get the documentation, use the following command

```
doxygen Doxygen.in
```

Use your favorite browser, to see the documentation

```
google-chrome documentation/html/index.html
firefox documentation/html/index.html
```
