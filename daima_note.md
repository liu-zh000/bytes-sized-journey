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