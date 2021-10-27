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

至于结果： ![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_1.png)

看起来好像是对的，但真的是对的？

试试渲染两个三角形，结果发现：

 ![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_Wrong_2.png)

问题出在哪？

问题在于处理了深度之后之后就不再进行深度检测而是直接写入导致的错误，然后想要试着改一下，结果：

 ![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_Wrong_1.png)

出现了莫名其妙的黑色填充，奇怪的 BUG 增加了。

感觉问题还是在于，最原本的深度值应该存储起来保证渲染的顺序是正常的，同时，要保证内部的颜色确实是守恒的。感觉黑边问题很可能出现在内部的颜色不守恒导致的，再加上这个渣机确实很难调试，明天在试着处理顺便优化一下 MSAA，希望能解决问题吧。



**2021-10-22**

差不多是解决了，首先是关于 msaa 的深度错误问题，这里直接把 msaa 的那个需要扩充出来的深度缓冲区去掉，然后每次检测到一个点要渲染的时候，就对他的四个子像素块进行深度的判断（这里也可以判断该点在不在三角形内，本题中场景比较简单所以这种简化方式也是可以的），然后对颜色进行乘除操作就可以。

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_Wrong_3.png)

然后说说这个黑色填充问题，实际上还是很简单的，因为判断的时候使用的是浮点数判断相等，浮点数判断相等工业界很难实现，所以只要判断的时候转成整数就可以规避了。

最后的结果：

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_2.png)

  感觉效果一般，不知道是本身就这样还是我的实现有问题，感觉实现的话，应该是没啥问题的...



关于msaa 效果一般的问题，只能说，因为用的是 2X msaa，所以采样点之后得到的结果就是这样，当然如果子像素使用顶点进行采样，得到的结果就会不一样：

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/02/msaa_3.png)

这样才能感觉到确实开启了 msaa。





**2021-10-24**

关于那个黑边的问题，经过各种方面的判断之后，这个黑边实际上并不是真正的黑色，而是一个颜色 / 4 之后得到的一个很像黑色的暗绿色。

第二章的所有问题基本就解决了。

------

## ***work 3***

第三章主要是光照和贴图，总的来说还是比较简单的，毕竟这个是之前最常用的一种东西。

首先说说这个模型，小奶牛的模型，什么都不加入渲染出来是这样的：

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/03/normal.png)





然后是对于光照模型，blinn-phong 模型已经是老朋友了，没必要多说，渲染的结果是这样的：

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/03/phong.png)

这里还要注意浮点数的精度问题，我们在 blinn-phong 模型中会用到半角向量，如果你使用正交化之后的光源向量（lightdir）和视角向量(viewdir)，就会出现如下的错误：

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/03/phong_wrong.png)

只能说是非常离谱，但是是否正交化在数学上实际上是没有太大的影响的，因此这里就先不正交化求得半角向量之后在进行正交化就可以获得正确的结果。



然后是 贴图，没少好说的感觉 ，uv 都给你了，直接代替漫反射系数就行了。

然后是 bump map 和 displacement map ，这个还是比较有意思的，涉及到了 TBN 的求法。

只需要注意，bump map 只是改变了 normal 来欺骗光照，模型本身实际上并没有发生改变。

displacement map 会直接改变模型的点，也常常用在对低模进行处理的贴图。



还想搞一个透视修正插值来着，有空的话搞一下吧，还有双线性插值啥的。

------

## ***work 4***

作业四相比之前的就简单了许多了，就是实现贝塞尔曲线，给的框架也十分简单。

实现的方法实际伤害还是挺多的，我们可以用二项展开来实现，不过这个方法比较麻烦，也可以用de Casteijau 算法来实现，这个方法是比较简单的，我们用两个数组的引用作为点然后每次循环更新直到找到最后一个点就结束。

这样不管有多少的控制点我们都可以绘制出来的，效果还是不错的。

![image](https://github.com/MoChen2001/Games101/tree/master/Photo/04/my_bezier_curve.png)

