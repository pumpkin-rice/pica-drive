# PicaDrive

基于 ODrive 的 FOC 代码，所有控制律代码全部用 C 重新实现，与 ODrive 原始代码相比已经面目全非。

为了实现兼容性和可移植性，尽量做到无外部依赖、基于标准 C。

## 功能列表
- [x] FOC 基础算法
- [x] 基于 PI 的电流环
- [ ] 基于 PI 的速度环
- [ ] 位置环
- [ ] 轨迹生成
  - [ ] 基于定加速度的二次位置曲线
  - [ ] 基于 jerk 的速度平滑曲线

## 第三方库

### matplotlib-cpp

使用 C++ 封装的 matplotlib-py，可以实现 C++ 绘图。

该库在编译时可能存在如下问题：

#### redefinition of 'struct matplotlibcpp::detail::select_npy_type<long long unsigned int>'

- 已知 mingw64 8.1 存在该问题

此时可以注释掉以下代码(`mex\pica-drive\third\matplotlib-cpp\matplotlibcpp.h`: 351)：
```cpp
// Sanity checks; comment them out or change the numpy type below if you're compiling on
// a platform where they don't apply
static_assert(sizeof(long long) == 8);
template <> struct select_npy_type<long long> { const static NPY_TYPES type = NPY_INT64; };
static_assert(sizeof(unsigned long long) == 8);
template <> struct select_npy_type<unsigned long long> { const static NPY_TYPES type = NPY_UINT64; };
```


