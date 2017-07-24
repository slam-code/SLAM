```c++
1),sudo apt-get install libeigen3-dev


2),默认安装路径是：/usr/include/eigen3 。  
此时不能直接编译，调整安装路径，将Eigen文件夹放在/usr/include 下面  
cd /usr/include/eigen3   
sudo cp Eigen/ .. -R  

或者：  
cd /usr/include/eigen3  
sudo cp Eigen -r /usr/include  


3)此时可正常编译代码。  

```