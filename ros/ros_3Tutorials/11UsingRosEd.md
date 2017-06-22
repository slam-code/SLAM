使用rosed编辑ROS中的文件

使用 rosed
rosed 是 rosbash 的一部分。利用它可以直接通过package名来获取到待编辑的文件而无需指定该文件的存储路径了。

使用方法:


$ rosed [package_name] [filename]
例子:


$ rosed roscpp Logger.msg
这个实例展示了如何编辑roscpp package里的Logger.msg文件。

如果该实例没有运行成功，那么很有可能是你没有安装vim编辑器。请参考编辑器部分进行设置。

如果文件名在package里不是唯一的，那么会呈现出一个列表，让你选择编辑哪一个文件。

使用Tab键补全文件名
使用这个方法，在不知道准确文件名的情况下，你也可以看到并选择你所要编辑的文件。

使用方法:


$ rosed [package_name] <tab>
编辑器
rosed默认的编辑器是vim。如果想要将其他的编辑器设置成默认的，你需要修改你的 ~/.bashrc 文件，增加如下语句:


export EDITOR='emacs -nw'
这将emacs设置成为默认编辑器。

注意: .bashrc文件的改变，只会在新的终端才有效。已经打开的终端不受环境变量的影响。

打开一个新的终端，看看那是否定义了EDITOR:


$ echo $EDITOR
emacs -nw
现在你已经成功设置并使用了rosed