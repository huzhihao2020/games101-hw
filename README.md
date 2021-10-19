# games101-hw
Assignments for GAMES101  
Course site: [games-101](https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html)

##### Lec07

* Painter's algorithm O(nlogn)
* **Z-Buffer** [assign2]
* **Blinn-Phong** Reflectance Model (**Diffuse** + Specular + Ambient)

##### Lec08 Shading2 

* Blinn-Phong Reflectance Model (Diffuse + **Specular + Ambient**)

  * 计算 Specular 项的时候，用半程向量与法线的夹角，估计 view 方向和镜面反射方向接受光线的差别。
* **Shading Frequencies** (Face + Vertex + Pixel)
* Read-time Rendering **Pipeline**

* **Texture Mapping**
  纹理的 uv 图

##### Lec09 Shading3

* Barycentric coordinates
  直到三角形顶点的属性，三角形中间的值都可以通过差值计算。 

* Applying textures

* **Texture Magnification**
  本质问题是差值结果是个非整数的 u,v 应该如何处理？

  * Nearest 直接四舍五入选最近的点
  * Bilinear 双线性差值，考虑最近的四个点
  * Bicubic

* **Texture Maginification(hard case)**

  如果纹理太大，会造成更严重的后果，通过超采样可以解决，但costly，引入Mipmap

* **Mipmap**
  Allow(fast, approx, squre)range queries. 

* **Anisotropic Filtering** 各向异性过滤

##### Lec10 Geometry
