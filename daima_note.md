# ��������¼ ˢ��ʼ�

## һ������

### ���ַ�
��ѭ����ʼ�ռ�ָ��ݲ�������Ķ��������߽紦��


1.�η�����

```cpp
{
    #include <cmath>
    double result = pow(10, i);  // ���� double 
    int intResult = static\_cast\<int>(pow(10, i));  // ǿ��ת��
}
```
2.�ַ�����ת��
![alt text](image.png)
```cpp

    string num1;
    `num1[i] - '0'`
```
3.ǰ׺�� + ���ֲ���
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
        // Ϊ�˷�����㣬�� size = n + 1 
        // sums[0] = 0 ��ζ��ǰ 0 ��Ԫ�ص�ǰ׺��Ϊ 0
        // sums[1] = A[0] ǰ 1 ��Ԫ�ص�ǰ׺��Ϊ A[0]
        // �Դ�����
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
4.��������
���ѭ��������ԭ��


## ��������


## ������ϣ��
�������������ж�һ��Ԫ���Ƿ���ּ����
���ǹ�ϣ��Ҳ�������˿ռ任ȡ��ʱ�䣬��Ϊ����Ҫʹ�ö�������飬set������map��������ݣ�����ʵ�ֿ��ٵĲ��ҡ�
�������������Ŀ��ʱ��������Ҫ�ж�һ��Ԫ���Ƿ���ֹ��ĳ���ҲӦ�õ�һʱ���뵽��ϣ����
```cpp
unordered_map<char, int> ��ʾ�洢 ??char��Ϊ��??��??int��Ϊֵ?? �Ĺ�ϣ��
#include <unordered_map>
std::unordered_map<char, int> mp; // ��map
mp.insert({'b', 98}); //mp['a'] = 97; 
// ֱ�ӷ��ʣ����������򴴽���
int val = mp['a']; 

// ��ȫ����//set2.count(num);
if (mp.find('z') != mp.end()) {
    int val = mp['z']; // ���ڴ���ʱ����
}

// ʹ��at()�������ڵļ����׳��쳣��
int safeVal = mp.at('a');

mp.erase('a');    // ɾ����Ϊ'a'��Ԫ��
if (auto it = mp.find('b'); it != mp.end()) {
    mp.erase(it); // ʹ�õ�����ɾ������ȫ
}

bool hasB = mp.contains('b');   // C++20��
bool hasA = (mp.find('a') != mp.end()); // �������а汾

// ʹ�÷�Χfor����
for (const auto& [key, value] : mp) {
    std::cout << key << ": " << value << "\n";
}

// ʹ�õ�����
for (auto it = mp.begin(); it != mp.end(); ++it) {
    char key = it->first;
    int value = it->second;
}

int size = mp.size();     // ��ֵ������
bool empty = mp.empty();  // �Ƿ�Ϊ��
mp.clear();               // ���map

auto it = hashtable.find(target - nums[i]);  // ���Ҳ����Ƿ����
if (it != hashtable.end()) {
    return {it->second, i};  // �����ҵ���ֵ
}
it��ָ���ҵ��ļ�ֵ�Եĵ�����
it->first��������ֵ��
it->second��ֵ��������
```
![alt text](image-6.png)


### ***---�ж�ѭ�����ÿ���ָ��---***

## string
```cpp
string s, string t
s.length()
sort(s.begin(), s.end());
sort(t.begin(), t.end());
return s==t;  // �����ַ��� s �� t �Ƿ���ȫ��ͬ
```
