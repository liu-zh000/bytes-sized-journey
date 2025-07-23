# 代码随想录 刷题笔记

## 一、数组

### 二分法
在循环中始终坚持根据查找区间的定义来做边界处理。


1.次方运算

```cpp
{
    #include <cmath>
    double result = pow(10, i);  // 返回 double 
    int intResult = static\_cast\<int>(pow(10, i));  // 强制转换
}
```
2.字符数字转换
![alt text](image.png)
```cpp

    string num1;
    `num1[i] - '0'`
```
3.前缀和 + 二分查找
```cpp
{
    class Solution {
public:
    int minSubArrayLen(int s, vector<int>& nums) {
        int n = nums.size();
        if (n == 0) {
            return 0;
        }
        int ans = INT_MAX;
        vector<int> sums(n + 1, 0); 
        // 为了方便计算，令 size = n + 1 
        // sums[0] = 0 意味着前 0 个元素的前缀和为 0
        // sums[1] = A[0] 前 1 个元素的前缀和为 A[0]
        // 以此类推
        for (int i = 1; i <= n; i++) {
            sums[i] = sums[i - 1] + nums[i - 1];
        }
        for (int i = 1; i <= n; i++) {
            int target = s + sums[i - 1];
            auto bound = lower_bound(sums.begin(), sums.end(), target);
            if (bound != sums.end()) {
                ans = min(ans, static_cast<int>((bound - sums.begin()) - (i - 1)));
            }
        }
        return ans == INT_MAX ? 0 : ans;
    }
};

}
```
4.螺旋矩阵
坚持循环不变量原则


## 二、链表