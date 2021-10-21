# **简介**



**2021-10-18**



games101 也算是快看完了，设计了很多基础但是重要的图形学的东西。

然后也是趁着还记得的这段时间做一下 games 的课后题，都是简单的东西，不会花太久的时间。

这里附上视频课程和课后作业的网址：

视频课程：https://www.bilibili.com/video/BV1X7411F744

课后作业：http://games-cn.org/forums/topic/allhw/

作业的全部框架都来自于课后作业。

-----

## **work  0**

算是个了解框架的东西，这里为了统一环境使用的时 Linux 的虚拟机，用起来还是很方便的，然后内部时就是简单的 C++ 代码，有 龙书的基础理解这个还是比较容易的。

---

## **work 1**

让写出三角形的旋转矩阵和投影矩阵。

照着公式直接搬参数就完事了。

注意这个框架使用的时列向量，这里虽然不会有太大的影响，但是如果想要自己拓展平移矩阵啥的就需要注意这个问题了。

旋转矩阵，围绕 ***z*** 轴的话就是简单的二维旋转矩阵的变化 ，二维是 [cos , -sin ;  sin ,cos ];

拓展到三位只有 ***y*** 轴的旋转符号会变到下面，其余的都变化不到。

这里确实是照搬公式就可以的，不过做到作业二的时候发现自己的透视矩阵有一点问题：

首先，这个整体环境是右手系的环境，而且，他的摄像机位置位于 Z 轴的正方向朝向反方向，所以导致投影矩阵有一点小小的变化（near 和 far 都要取负）。

投影矩阵的推导等等还是挺有意思的，课程讲的理解方式也比很多书中的提出来的理解方式更容易让人理解，这里附上一个感觉写的不错的推导过程可以看一下：

https://zhuanlan.zhihu.com/p/122411512

这里对于透视投影第三行的推导还是有点问题的，实际上由更简单的方式，直接列出两个方程就可以，不必要做过多的说明。



**2021-10-19**



推导出来之后，得到的结果跟我们常用的公式的结果是不一样的，我们常用的只有四个参数，可以直接根据现在的公式推导出经常用的公式，简单的几何问题，这里不再多说了。

---

## ***work 2***



这个作业算是做了一半了把，完成了一下基础。

基础目标是栅格化三角形，进阶目标是完成 SSAA 和 MSAA。 

首先说基础目标，实际上挺简单的，我们要修改两个函数，栅格化三角形以及判断点是不是在三角形之内的函数。

对于判断点是不是在三角形之内的函数，就是叉积的运用，没啥好说的。

栅格化三角形，得到包围盒之后进行逐个像素的判断就可以了，感觉也没啥好说的。

这里把三角形的中心坐标插值已经给你了，所以难点已经没了，也就没啥好说的了，可能唯一要注意的就是深度值。

深度值由于摄像机是向 Z 轴负向观察的，因此注意一下就可以了。

得到的结果大概是这样的：

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/rasterizer.png)



**2021-10-21**



SSAA 写出来之后运行一下，然后虚拟机直接暴毙了，说明了这渣机确实该换了，也说明了 SSAA 的开销是真的大，SSAA 渲染两个三角新实在是扛不住，这里就勉强渲染一个三角形了，得到的效果还不错，

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/ssaa.png)





然后是 MSAA，相对于 SSAA 来说开销就要小很多，效果也是不错的，开销会比较小，少了四倍像素的的存储和采样，因为是 CPU计算，所以实际上还少了一堆向量的计算，这个对于 CPU 来说开销也是巨大的。

至于结果： ![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa.png)

看起来好像是对的，但真的是对的？

试试渲染两个三角形，结果发现：

 ![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_Wrong1.png)

问题出在哪？

问题在于处理了深度之后之后就不再进行深度检测而是直接写入导致的错误，然后想要试着改一下，结果：

 ![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_Wrong2.png)

出现了莫名其妙的黑色填充，奇怪的 BUG 增加了。

感觉问题还是在于，最原本的深度值应该存储起来保证渲染的顺序是正常的，同时，要保证内部的颜色确实是守恒的。感觉黑边问题很可能出现在内部的颜色不守恒导致的，再加上这个渣机确实很难调试，明天在试着处理顺便优化一下 MSAA，希望能解决问题吧。