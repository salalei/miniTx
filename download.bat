@REM sunxi-fel.exe -p write 0x00000000 tina-spl.bin
@REM sunxi-fel.exe exec 0x00000000
@REM sunxi-fel.exe -p write 0x80000000 rtthread.bin
@REM sunxi-fel.exe exec 0x80000000
sunxi-fel.exe -p spl boot.bin
sunxi-fel.exe -p write 0x80000000 rtthread.bin
sunxi-fel.exe exec  0x80000000