#!/bin/bash
 # echo of all files in a directory

for file in *.STL
do
  name=${file%%[.]*}
  #meshlabserver -i $file -o $name'.stl' -s scale.mlx -om vn
  #meshlabserver -i $file -o $name'.obj' -s scale.mlx -om vn
  meshlabserver -i $file -o $name'.obj' -om vn
  echo $name
done
