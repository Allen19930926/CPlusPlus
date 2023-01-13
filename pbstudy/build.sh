rm a.out
rm *pb*
CUR_DIR=$(cd `dirname $0`; pwd)
echo $CUR_DIR
protoc --cpp_out=$CUR_DIR  *.proto
g++ main.cpp *.pb.cc -std=c++11 -lprotobuf -lpthread
./a.out