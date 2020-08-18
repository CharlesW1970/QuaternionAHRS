四元数姿态解算算法，原理详见：https://www.cnblogs.com/HongxiWong/p/12357230.html与https://www.cnblogs.com/HongxiWong/p/12398197.html

由于运动加速度会对加速度计测量产生影响，即超重或失重状态下的加速度计测量结果不能准确反应实际重力加速度，从而影响结果精度。故该算法根据加速度计模长动态降低加速度计补偿值权重，从而减小运动加速度对结果的影响。用户可通过改变MinACC与MaxACC的值自行调节加速度有效范围。