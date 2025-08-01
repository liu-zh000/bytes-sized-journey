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


## 三、哈希表
都是用来快速判断一个元素是否出现集合里。
但是哈希法也是牺牲了空间换取了时间，因为我们要使用额外的数组，set或者是map来存放数据，才能实现快速的查找。
如果在做面试题目的时候遇到需要判断一个元素是否出现过的场景也应该第一时间想到哈希法！
```cpp
unordered_map<char, int> 表示存储 ??char作为键??、??int作为值?? 的哈希表
#include <unordered_map>
std::unordered_map<char, int> mp; // 空map
mp.insert({'b', 98}); //mp['a'] = 97; 
// 直接访问（若不存在则创建）
int val = mp['a']; 

// 安全访问//set2.count(num);
if (mp.find('z') != mp.end()) {
    int val = mp['z']; // 仅在存在时访问
}

// 使用at()（不存在的键会抛出异常）
int safeVal = mp.at('a');

mp.erase('a');    // 删除键为'a'的元素
if (auto it = mp.find('b'); it != mp.end()) {
    mp.erase(it); // 使用迭代器删除更安全
}

bool hasB = mp.contains('b');   // C++20起
bool hasA = (mp.find('a') != mp.end()); // 兼容所有版本

// 使用范围for遍历
for (const auto& [key, value] : mp) {
    std::cout << key << ": " << value << "\n";
}

// 使用迭代器
for (auto it = mp.begin(); it != mp.end(); ++it) {
    char key = it->first;
    int value = it->second;
}

int size = mp.size();     // 键值对数量
bool empty = mp.empty();  // 是否为空
mp.clear();               // 清空map

auto it = hashtable.find(target - nums[i]);  // 查找补数是否存在
if (it != hashtable.end()) {
    return {it->second, i};  // 返回找到的值
}
it：指向找到的键值对的迭代器
it->first：键（数值）
it->second：值（索引）
```
![alt text](image-6.png)


### ***---判断循环就用快慢指针---***

## string
```cpp
string s, string t
s.length()
sort(s.begin(), s.end());
sort(t.begin(), t.end());
return s==t;  // 返回字符串 s 和 t 是否完全相同
```
