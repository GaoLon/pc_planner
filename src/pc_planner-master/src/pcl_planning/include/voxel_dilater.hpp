/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
//ijk为一会儿会用的偏移量,x,y,z为当前位置, sy为x方向栅格总数, sz为xy一层的栅格总数,bxbybz为地图xyz边界数,   
//ck为bool即check本身这个点是否到边界。检查他的26个邻居，只有满足俩条才能把这个邻居当表面点pushback
//1.（没到边界）或者（到边界了 但是 这个邻居在界内）
//2. 这个邻居是free的
// ogm为存放occ free dialte信息的数组即voxels,  
//ofst一会儿会用即idx, val=2即Dilated, fdl为std::vector<Eigen::Vector3i>存放表面的栅格即cvec


#ifndef VOXEL_DILATER 
#define VOXEL_DILATER(i, j, k, x, y, z, sy, sz, bx, by, bz, ck, ogm, ofst, val, fdl)                                                                                                                                                      \
(ck) = ((x) == 0 || (x) == (bx) || (y) == 0 || (y) == (by) || (z) == 0 || (z) == (bz));                                                                                                                                                   \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                                   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                                )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }
#endif



