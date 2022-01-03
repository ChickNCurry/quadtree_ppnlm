#include <fstream>

using namespace std;

class BlobReader {
public:
    BlobReader(const string& filename) : f(filename, ios::in | ios::binary) {}

    template <typename Type>
    typename enable_if<is_standard_layout<Type>::value, BlobReader&>::type
    operator >> (Type& Element) {
        Read(&Element, 1);
        return *this;
    }

    // CAUTION: This function may break down on big-endian architectures.
    //          The ordering of bytes has to be reverted then.
    template <typename T>
    void Read(T* Dest, size_t Size) {
        f.read(reinterpret_cast<char*>(Dest), Size * sizeof(T));
    }

    bool isValid() const {
        return (bool)(f);
    }

private:
    ifstream f;
};

class BlobWriter {
public:
    BlobWriter(const std::string& filename): f(filename, ios::out | ios::binary) {}

    template <typename Type>
    typename std::enable_if<std::is_standard_layout<Type>::value, BlobWriter&>::type
    operator << (Type& Element) {
        Write(&Element, 1);
        return *this;
    }

    // CAUTION: This function may break down on big-endian architectures.
    //          The ordering of bytes has to be reverted then.
    template <typename T>
    void Write(T* Src, size_t Size) {
        f.write(reinterpret_cast<char*>(Src), Size * sizeof(T));
    }

private:
    ofstream f;
};

