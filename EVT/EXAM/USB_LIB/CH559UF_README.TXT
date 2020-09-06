The interface description and example program of the U disk file-level subroutine library of CH559
Please refer to the CH559 evaluation board information and related PDF documents

************************************************** ***************************************

CH559UFI(small)/CH559UFX(large) V1.0 latest update 2015.01.22

************************************************** ***************************************

CH559UFI(small)/CH559UFX(large) V1.1 latest update 2015.12.23

1. Fix BUG
 1. CH559FileWrite failed to write root directory length limit
1. Updates and additions
 1. Update DMA to support large sectors of 4096 bytes

************************************************** ***************************************

CH559UFI(small)/CH559UFX(large) V1.2 latest update 2016.02.18

1. Fix BUG
 1. CH559FileModify file date and time are wrong

************************************************** ***************************************

CH559UFR(small & large) V1.0 latest update 2017.05.22
  CH559UFR is a simplified version of "File Read Only" based on CH559UFI. The main differences are:
    1. Only support file byte read operation, delete write operation, move file pointer, query remaining capacity, sector operation, etc.
    2. CH559UFR itself occupies no more than 6KB of code space, which saves 4KB of code space than CH559UFI
    3. Use CH559UFR.H for the header file instead, except for the header file, other usage methods are the same

************************************************** ***************************************

CH559UFI(small)/CH559UFX(large)/CH559UFR(read only @small & large) V1.3 latest update 2017.09.05

1. Fix BUG
 1. CH559ByteRead large file error

************************************************** ***************************************
