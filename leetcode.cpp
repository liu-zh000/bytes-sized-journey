
 #include <string>
 #include <iostream>
 #include <vector>
 using namespace std;
// class Solution {
// public:
//     string multiply(string num1, string num2) {
//         int a = num1.size();//123
//         int b = num2.size();
//         int cheng10 = 1;
//         int all1 = 0 ;
//         int cheng = 0;
//         int numn1i = 0 ; int  numn2i = 0;
//         vector<int> res ;
//         //string res = "";
//         int m = 0 ;int jinwei = 0;
//         for(int i = b-1 ;i>=0 ; i--)//456
//         {
            
//             for(int j = a-1 ;j >=0 ; j--)
//             {
//                 cheng = (num2[i] - '0') * (num1[j] - '0');
//                 cheng = cheng * cheng10;
//                 cheng10 = 10 * cheng10;
//                 all1 = all1 + cheng;
//                 //res[m] = cheng%10 + jinwei;
//                 //jinwei = cheng/10;  
//                 //m++;
//             }
//             res.push_back(all1);
//             all1 = 0;
//             cheng10 = 1;
//         }

//         int sum = 0 ;
//         for(int l = 0; l <= res.size() -1  ; l++ )
//         {
//             sum  = sum + res[l]*cheng10;
//              cheng10 = 10 * cheng10;
//         }

        
//         return to_string(sum);
//     }
// };

class Solution {
public:
    int minSubArrayLen(int target, vector<int>& nums) {
        int largem = nums[0]; int ptr = 0;int sum =0;
        int l = 1;
        for (int i = 0 ; i < nums.size() ; i ++ )
        {
            if(largem < nums[i])
            {
                largem = nums[i];
                ptr = i ;
            }
        }

        if (largem >= target)
        {
            return 1;
        }
        else
        {
            sum = largem;
            int ptr2 = ptr ;
            while(sum < target)
            {

                if(ptr2 - 1 < 0  && ptr + 1 >= nums.size())
                {
                    return 0;
                }

                if(ptr + 1 < nums.size() || ptr2 - 1 >= 0)
                {
                    if(nums[ptr + 1] >= nums[ptr2 - 1])
                    {
                        sum += nums[ptr + 1 ];
                        ptr ++ ;
                        l++;
                    }
                    else
                    {
                        sum += nums[ptr2 - 1 ];
                        ptr2 --;
                        l++;
                    }
                }
                else if(ptr + 1 >= nums.size() && ptr2 - 1 >=  0)
                {
                    sum += nums[ptr2 - 1 ];
                        ptr2 --;
                        l++;
                }
                else if(ptr2 - 1 <  0)
                {
                    sum += nums[ptr + 1 ];
                        ptr ++ ;
                        l++;
                }

            }
            return  l; 

        }
     }
};

int main()
{
    Solution a;
    vector<int> ab ={1,1,1,1,1,1,1,1};
    int result = a.minSubArrayLen(11 , ab);
    std::cout << "½á¹û"<<result << std::endl;
}

// class Solution {
// public:
//     string multiply(string num1, string num2) {
//         int a = num1.size();
//         int b = num2.size();
//         int numn1i = 0 ; int numn2i = 0;
//         for(int i = 0 ;i<=a-1 ; i++)//123
//         {
//             numn1i = numn1i * 10 + int(num1[i] - '0') ;
//         }

//         for(int j = 0 ;j <=b-1 ; j++)
//         {
//             numn2i = numn2i * 10 + int(num2[j] - '0' );
//         }

//         return to_string(numn1i * numn2i);
//     }
// };