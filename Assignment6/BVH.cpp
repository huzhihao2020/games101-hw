#include <algorithm>
#include <cassert>
#include <vector>
#include <chrono>
#include <map>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::milliseconds milliseconds;
    Clock::time_point t0 = Clock::now();

    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);
    // root = recursiveBuildSAH(primitives);

    Clock::time_point t1 = Clock::now();
    milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);
    // std::cout << ms.count() << "ms\n";

    if (splitMethod == SplitMethod::NAIVE)
    {
        std::cout << "\rBVH Generation(NAIVE) complete: \nTime Taken: " << ms.count() << "ms" << std::endl;
        // printf("\rBVH Generation(NAIVE) complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
    }
    if (splitMethod == SplitMethod::SAH)
    {
        std::cout << "\rBVH Generation(SAH) complete: \nTime Taken: " << ms.count() << "ms" << std::endl;
        // printf("\rBVH Generation(NAIVE) complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
    }
}

// BVHBuildNode *BVHAccel::recursiveBuild_my(std::vector<Object *> objects)
// {
//     BVHBuildNode *node = new BVHBuildNode();

//     // Compute bounds of all primitives in BVH node
//     int mincostID = 0;
//     int minCostObjId = objects.size() / 2;

//     Bounds3 bounds;
//     for (int i = 0; i < objects.size(); ++i)
//         bounds = Union(bounds, objects[i]->getBounds());
//     if (objects.size() == 1)
//     {
//         // Create leaf _BVHBuildNode_
//         node->bounds = objects[0]->getBounds();
//         node->object = objects[0];
//         node->left = nullptr;
//         node->right = nullptr;
//         return node;
//     }
//     else if (objects.size() == 2)
//     {
//         node->left = recursiveBuild(std::vector{objects[0]});
//         node->right = recursiveBuild(std::vector{objects[1]});

//         node->bounds = Union(node->left->bounds, node->right->bounds);
//         return node;
//     }
//     else
//     {
//         switch (splitMethod)
//         {
//         case SplitMethod::NAIVE:
//         {
//             Bounds3 centroidBounds;
//             for (int i = 0; i < objects.size(); ++i)
//                 centroidBounds =
//                     Union(centroidBounds, objects[i]->getBounds().Centroid());
//             int dim = centroidBounds.maxExtent();
//             switch (dim)
//             {
//             case 0:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
//                           { return f1->getBounds().Centroid().x <
//                                    f2->getBounds().Centroid().x; });
//                 break;
//             case 1:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
//                           { return f1->getBounds().Centroid().y <
//                                    f2->getBounds().Centroid().y; });
//                 break;
//             case 2:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
//                           { return f1->getBounds().Centroid().z <
//                                    f2->getBounds().Centroid().z; });
//                 break;
//             }
//         }
//         break;

//         case SplitMethod::SAH:
//         {
//             Bounds3 centroidBounds;
//             for (int i = 0; i < objects.size(); ++i)
//                 centroidBounds =
//                     Union(centroidBounds, objects[i]->getBounds().Centroid());
//             int dim = centroidBounds.maxExtent();
//             switch (dim)
//             {
//             case 0:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
//                           { return f1->getBounds().Centroid().x <
//                                    f2->getBounds().Centroid().x; });
//                 break;
//             case 1:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
//                           { return f1->getBounds().Centroid().y <
//                                    f2->getBounds().Centroid().y; });
//                 break;
//             case 2:
//                 std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
//                           { return f1->getBounds().Centroid().z <
//                                    f2->getBounds().Centroid().z; });
//                 break;
//             }
//             // 前面和NAIVE一样, 确定最长的轴，并将objects按轴上位置排好序
//             // 应用SAH需要在划分前计算各种划分对应的Cost
//             // Cost = C_trav + NA*SA/SN*C_inter + NB*SB/SN*C_inter
//             // Assume C_trav and C_inter are both constant 1
//             Bounds3 boundsA;
//             Bounds3 boundsB;
//             double minCost = std::numeric_limits<float>::infinity();
//             // 分buckets并初始化, bucketsNum<32
//             int bucketsNum = 12;
//             std::vector<Bounds3> bucketsBounds(bucketsNum, Bounds3());
//             std::vector<int> bucketsCounts(bucketsNum, 0);
//             std::vector<int> BucketsBeginObjId(bucketsNum, -1);
//             // 遍历objects, 将他们填入相应的buckets
//             for (int i = 0; i < objects.size(); i++)
//             {
//                 auto center_vec =
//                     centroidBounds.Offset(objects[i]->getBounds().Centroid()); // bounds还是centroidBounds?
//                 int center_bucketsID = dim == 0   ? bucketsNum * center_vec.x
//                                        : dim == 1 ? bucketsNum * center_vec.y
//                                                   : bucketsNum * center_vec.z;

//                 if (BucketsBeginObjId[center_bucketsID] == -1)
//                     BucketsBeginObjId[center_bucketsID] = i;

//                 bucketsBounds[center_bucketsID] =
//                     Union(bucketsBounds[center_bucketsID], objects[i]->getBounds().Centroid());
//                 bucketsCounts[center_bucketsID]++;
//             }

//             //计算每个bucket的cost
//             for (int j = 1; j < bucketsNum; j++)
//             {
//                 Bounds3 A, B;
//                 int NA = 0;
//                 int NB = 0;
//                 for (int k = 0; k < j; k++)
//                 {
//                     A = Union(A, bucketsBounds[k]);
//                     NA += bucketsCounts[k];
//                 }

//                 for (int k = j; k < bucketsBounds.size(); k++)
//                 {
//                     B = Union(B, bucketsBounds[k]);
//                     NB += bucketsCounts[k];
//                 }

//                 double SA = A.SurfaceArea();
//                 double SB = B.SurfaceArea();
//                 double SN = centroidBounds.SurfaceArea();

//                 double cost = 1.0f + (SA * NA + SB * NB) / SN;
//                 if (cost < minCost)
//                 {
//                     minCost = cost;
//                     mincostID = j;
//                     minCostObjId = BucketsBeginObjId[j];
//                 }
//             }
//         }
//         break;
//         default:
//             printf("select splitMode\n");
//             break;
//         }

//         auto beginning = objects.begin();
//         auto middling = objects.begin() + minCostObjId; //修改划分位置,需要找到划分的bucketID中第一个物体的Index
//         auto ending = objects.end();

//         auto leftshapes = std::vector<Object *>(beginning, middling); //左闭右开
//         auto rightshapes = std::vector<Object *>(middling, ending);

//         assert(objects.size() == (leftshapes.size() + rightshapes.size()));

//         node->left = recursiveBuild(leftshapes);
//         node->right = recursiveBuild(rightshapes);

//         node->bounds = Union(node->left->bounds, node->right->bounds);
//     }

//     return node;
// }

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        //算出最大的包围盒（通用的，因为两个方法都要用到）
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());

        std::vector<Object*> leftshapes;
        std::vector<Object*> rightshapes;

            switch (splitMethod)//这里注意了在BVH.h里面有个枚举类，构造函数中的初始将决定是普通方法，还是SAH
            {
            case SplitMethod::NAIVE:
            {
                int dim = centroidBounds.maxExtent();//算出最大的跨度对应的值，x为0，y为1，z为2
                int index = objects.size() / 2;
                switch (dim)
                //排序，针对最大的跨度排序
                {
                case 0:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x <
                               f2->getBounds().Centroid().x;
                    });
                    break;
                case 1:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y <
                               f2->getBounds().Centroid().y;
                    });
                    break;
                case 2:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z <
                               f2->getBounds().Centroid().z;
                    });
                    break;
                }

                auto beginning = objects.begin();
                auto middling = objects.begin() + index;
                auto ending = objects.end();
                 //递归算法，枢轴是中间元素。
                leftshapes = std::vector<Object *>(beginning, middling);
                rightshapes = std::vector<Object *>(middling, ending);
            }
            break;
            case SplitMethod::SAH:
            {
                float nArea = centroidBounds.SurfaceArea();//算出最大的

                int minCostCoor = 0;
                int mincostIndex = 0;
                float minCost = std::numeric_limits<float>::infinity();
                std::map<int, std::map<int, int>> indexMap;
                //indexmap用于记录x，y，z（前一个int代表x，y，z，后一个map代表那个轴对应的map）
                //遍历x，y，z的划分
                for(int i = 0; i < 3; i++)
                {
                    int bucketCount = 12;//桶的个数，这里定了12个桶，就是在某一个轴上面划分了12个区域
                    std::vector<Bounds3> boundsBuckets;
                    std::vector<int> countBucket;
                    for(int j = 0; j < bucketCount; j++)
                    {
                        boundsBuckets.push_back(Bounds3());
                        countBucket.push_back(0);
                    }

                    std::map<int, int> objMap;

                    for(int j = 0; j < objects.size(); j++)
                    {
                        int bid =  bucketCount * (centroidBounds.Offset(objects[j]->getBounds().Centroid()))[i];//算出对应x，y。z上的id值，这里【i】代表x，y，z
                        if(bid > bucketCount - 1)//实质是可以划分13个区域的，将最后一个区域合并。
                        {
                            bid = bucketCount - 1;
                        }
                        Bounds3 b = boundsBuckets[bid];
                        b = Union(b, objects[j]->getBounds().Centroid());
                        boundsBuckets[bid] = b;
                        countBucket[bid] = countBucket[bid] + 1;
                        objMap.insert(std::make_pair(j, bid));
                    }

                    indexMap.insert(std::make_pair(i, objMap));
                    //对于每一个划分，计算他所对应的花费，方法是对于桶中的每一个面积，计算他的花费，最后进行计算
                    //找出这个划分。
                    for(int j = 1; j < boundsBuckets.size(); j++)
                    {
                        Bounds3 A;
                        Bounds3 B;
                        int countA = 0;
                        int countB = 0;
                        for(int k = 0; k < j; k++)
                        {
                            A = Union(A, boundsBuckets[k]);
                            countA += countBucket[k];
                        }

                        for(int k = j; k < boundsBuckets.size(); k++)
                        {
                            B = Union(B, boundsBuckets[k]);
                            countB += countBucket[k];
                        }

                        float cost = 1 + (countA * A.SurfaceArea() + countB * B.SurfaceArea()) / nArea;//计算花费
                        //找出这个花费。
                        if(cost < minCost)
                        {
                            minCost = cost;
                            mincostIndex = j;
                            minCostCoor = i;
                        }
                    }
                }
                //加入左右数组，这里很重要，具体还是看那篇博客
                for(int i = 0; i < objects.size(); i++)
                {
                    if(indexMap[minCostCoor][i] < mincostIndex)
                    {
                        leftshapes.push_back(objects[i]);
                    }
                    else
                    {
                        rightshapes.push_back(objects[i]);
                    }
                }
            }
            break;
            default:
            break;
        }

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        //递归计算，同普通方法
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}
Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // TODO Traverse the BVH to find intersection

    // 1. check if ray hits box of current node
    Intersection inter, hit1, hit2;
    std::array<int, 3> dirIsNeg =
        {int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0)};

    // ray hit node.bbox?
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        // std::cout << ray << "not intersected" << std::endl;
        return inter;
    }

    // hit, check if node is leaf node
    if ((node->left == nullptr) && (node->right == nullptr))
    {
        // ray intersect with all objects in the node
        //  观察定义发现，一个leaf只有一个object
        inter = node->object->getIntersection(ray);
        // inter.obj = node->object;
        // std::cout << "hit objects" << std::endl;
    }
    else
    {
        hit1 = getIntersection(node->left, ray);
        hit2 = getIntersection(node->right, ray);
        inter = hit1.distance < hit2.distance ? hit1 : hit2;
        // std::cout << "not leaf" << std::endl;
    }
    return inter;
}