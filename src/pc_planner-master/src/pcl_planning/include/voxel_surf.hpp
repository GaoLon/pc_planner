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
//这个看上下左右前后六个集角块，有一个空的就说明是surface


#ifndef VOXEL_surf 
#define VOXEL_surf(i, j, k, x, y, z, sy, sz, bx, by, bz, ck, ogm, ofst, fdl)                                                                                                                           \
(ck) = ((x) == 0 || (x) == (bx) || (y) == 0 || (y) == (by) || (z) == 0 || (z) == (bz));                                                                                                                           \
(i) = (x) - 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                                   )) && (ogm)[(ofst)] == 0) {  (fdl).emplace_back((x), (y), (z)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0                    )) && (ogm)[(ofst)] == 0) {  (fdl).emplace_back((x), (y), (z)); }  \
(i) = (x);     (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) >= 0   )) && (ogm)[(ofst)] == 0) {  (fdl).emplace_back((x), (y), (z));  }  \
(i) = (x);     (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) <= (bz))) && (ogm)[(ofst)] == 0) {  (fdl).emplace_back((x), (y), (z));  }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) {  (fdl).emplace_back((x), (y), (z));  }  \
(i) = (x) + 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                                )) && (ogm)[(ofst)] == 0) {  (fdl).emplace_back((x), (y), (z));  }  
#endif



