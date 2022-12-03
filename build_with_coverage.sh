mkdir -p build/
cd build
rm -rf *
cmake -DCOVERAGE=ON -DCMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
./test/cpp-test
