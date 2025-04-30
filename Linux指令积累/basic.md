# Linux 指令积累

## 本地机与远程机之间进行文件拷贝
1. scp -r [源目录] [目标路径]
递归地拷贝整个目录的命令，scp(secure copy, 安全拷贝)的一种用法，它基于 SSH 协议，常用于在本地与远程机器之间安全传输文件。
```bash
// 从本地复制整个目录到远程服务器
scp -r ./myfolder user@remote_ip:/home/user/

// 从远程服务器复制整个目录到本地
scp -r user@remote_ip:/home/user/myfolder ./local_backup/
```

## 打包/解压
1. 打包tar
tar [选项] 打包后的文件名 要打包的文件/目录
常用选项

| 选项   |  说明      |
| :-------| :-------|
| -c     | 创建新的打包文件 creat |
| -x    | 解打包 extract|
| -f    | 指定打包文件名(必须放在最后) |
| -v    | 显示详细过程 verbose |
| -z    | 使用gzip压缩或解压   |
| -j    | 使用 bzip2 压缩或解压 |
| -J    | 使用xz 压缩或解压 |
| -t    | 查看打包内容 |
| -C    | 指定解压目录 |

示例
```bash
# 打包目录不压缩
tar -cvf archive.tar /path/to/directory

tar -xvf archive.tar # 解打包

tar -xvf archive.tar -C /target/directory  # 解打包到指定目录
```

2. 压缩与解压
**gzip压缩**
```bash
# 压缩文件（会删除原文件，生成.gz文件）
gzip filename

# 解压.gz文件
gzip -d filename.gz
# 或
gunzip filename.gz

# 压缩时保留原文件
gzip -c filename > filename.gz

# 显示压缩信息
gzip -l filename.gz
```

**bzip2压缩**(比gzip压缩率高)
```bash
# 压缩文件
bzip2 filename

# 解压.bz2文件
bzip2 -d filename.bz2
# 或
bunzip2 filename.bz2

# 压缩时保留原文件
bzip2 -c filename > filename.bz2
```

**xz压缩**(更高压缩率)
```bash
# 压缩文件
xz filename

# 解压.xz 文件
xz -d filename.xz
# 或者
unxz filename.xz

# 压缩时保留原文件
xz -c filename > filename.xz
```

3. 打包并压缩
* gzip
```bash
tar -czvf archive.tar.gz /path/to/directory

tar -xzvf archive.tar.gz
```
* bzip2
```bash
tar -cjvf archive.tar.bz2 /path/to/directory

tar -xjvf archive.tar.bz2
```

* xz
```bash
tar -cJvf archive.tar.xz /path/to/directory

tar -xJvf archive.tar.xz
```