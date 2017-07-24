lua_getallocf
lua_Alloc lua_getallocf (lua_State *L, void **ud);
返回给定状态机的内存分配器函数。如果 ud 不是 NULL ，Lua 把调用lua_newstate 时传入的那个指针放入*ud 。

lua_getfenv
void lua_getfenv (lua_State *L, int index);
把索引处值的环境表压入堆栈。

lua_getfield
void lua_getfield (lua_State *L, int index, const char *k);
把 t[k] 值压入堆栈，这里的 t 是指有效索引 index 指向的值。在 Lua 中，这个函数可能触发对应 "index" 事件的元方法（参见 §2.8）。


lua_getglobal
void lua_getglobal (lua_State *L, const char *name);
把全局变量 name 里的值压入堆栈。这个是用一个宏定义出来的：
     #define lua_getglobal(L,s)  lua_getfield(L, LUA_GLOBALSINDEX, s)
lua_getmetatable
int lua_getmetatable (lua_State *L, int index);
把给定索引指向的值的元表压入堆栈。如果索引无效，或是这个值没有元表，函数将返回 0 并且不会向栈上压任何东西。
lua_gettable
void lua_gettable (lua_State *L, int index);
把 t[k] 值压入堆栈，这里的 t 是指有效索引 index 指向的值，而 k 则是栈顶放的值。
这个函数会弹出堆栈上的 key （把结果放在栈上相同位置）。在 Lua 中，这个函数可能触发对应 "index" 事件的元方法（参见§2.8）。
lua_gettop
int lua_gettop (lua_State *L);
返回栈顶元素的索引。因为索引是从 1 开始编号的，所以这个结果等于堆栈上的元素个数（因此返回 0 表示堆栈为空）。
lua_insert
void lua_insert (lua_State *L, int index);
把栈顶元素插入指定的有效索引处，并依次移动这个索引之上的元素。不要用伪索引来调用这个函数，因为伪索引不是真正指向堆栈上的位置。
lua_Integer
typedef ptrdiff_t lua_Integer;
这个类型被用于 Lua API 接收整数值。
缺省时这个被定义为 ptrdiff_t ，这个东西通常是机器能处理的最大整数类型。
lua_isboolean
int lua_isboolean (lua_State *L, int index);
当给定索引的值类型为 boolean 时，返回 1 ，否则返回 0 。
lua_iscfunction
int lua_iscfunction (lua_State *L, int index);
当给定索引的值是一个 C 函数时，返回 1 ，否则返回 0 。
lua_isfunction
int lua_isfunction (lua_State *L, int index);
当给定索引的值是一个函数（ C 或 Lua 函数均可）时，返回 1 ，否则返回 0 。
lua_islightuserdata
int lua_islightuserdata (lua_State *L, int index);
当给定索引的值是一个 light userdata 时，返回 1 ，否则返回 0 。
lua_isnil
int lua_isnil (lua_State *L, int index);
当给定索引的值是 nil 时，返回 1 ，否则返回 0 。
lua_isnumber
int lua_isnumber (lua_State *L, int index);
当给定索引的值是一个数字，或是一个可转换为数字的字符串时，返回 1 ，否则返回 0 。
lua_isstring
int lua_isstring (lua_State *L, int index);
当给定索引的值是一个字符串或是一个数字（数字总能转换成字符串）时，返回 1 ，否则返回 0 。
lua_istable
int lua_istable (lua_State *L, int index);
当给定索引的值是一个 table 时，返回 1 ，否则返回 0 。
lua_isthread
int lua_isthread (lua_State *L, int index);
当给定索引的值是一个 thread 时，返回 1 ，否则返回 0 。
lua_isuserdata
int lua_isuserdata (lua_State *L, int index);
当给定索引的值是一个 userdata （无论是完整的 userdata 还是 light userdata ）时，返回 1 ，否则返回 0 。
lua_lessthan
int lua_lessthan (lua_State *L, int index1, int index2);
如果索引 index1 处的值小于索引 index2 处的值时，返回 1 ；否则返回 0 。其语义遵循 Lua 中的< 操作符（就是说，有可能调用元方法）。如果任何一个索引无效，也会返回 0 。
lua_load
int lua_load (lua_State *L,
              lua_Reader reader,
              void *data,
              const char *chunkname);
加载一个 Lua chunk 。如果没有错误， lua_load 把一个编译好的 chunk 作为一个 Lua 函数压入堆栈。否则，压入出错信息。 lua_load 的返回值可以是：
0: 没有错误；
LUA_ERRSYNTAX: 在预编译时碰到语法错误；
LUA_ERRMEM: 内存分配错误。
这个函数仅仅加栽 chunk ；而不会去运行它。
lua_load 会自动检测 chunk 是文本的还是二进制的，然后做对应的加载操作（参见程序luac）。
lua_load 函数使用一个用户提供的reader 函数来读取 chunk （参见lua_Reader）。data 参数会被传入读取器函数。
chunkname 这个参数可以赋予 chunk 一个名字，这个名字被用于出错信息和调试信息（参见§3.8）。
lua_newstate
lua_State *lua_newstate (lua_Alloc f, void *ud);
创建的一个新的独立的状态机。如果创建不了（因为内存问题）返回 NULL 。参数 f 是一个分配器函数； Lua 将通过这个函数做状态机内所有的内存分配操作。第二个参数 ud ，这个指针将在每次调用分配器时被直接传入。
lua_newtable
void lua_newtable (lua_State *L);
创建一个空 table ，并将之压入堆栈。它等价于 lua_createtable(L, 0, 0) 。
lua_newthread
lua_State *lua_newthread (lua_State *L);
创建一个新线程，并将其压入堆栈，并返回维护这个线程的 lua_State 指针。这个函数返回的新状态机共享原有状态机中的所有对象（比如一些 table），但是它有独立的执行堆栈。
没有显式的函数可以用来关闭或销毁掉一个线程。线程跟其它 Lua 对象一样是垃圾收集的条目之一。
lua_newuserdata
void *lua_newuserdata (lua_State *L, size_t size);
这个函数分配分配一块指定大小的内存块，把内存块地址作为一个完整的 userdata 压入堆栈，并返回这个地址。
userdata 代表 Lua 中的 C 值。完整的 userdata 代表一块内存。它是一个对象（就像 table 那样的对象）：你必须创建它，它有着自己的元表，而且它在被回收时，可以被监测到。一个完整的 userdata 只和它自己相等（在等于的原生作用下）。
当 Lua 通过 gc 元方法回收一个完整的 userdata 时， Lua 调用这个元方法并把 userdata 标记为已终止。等到这个 userdata 再次被收集的时候，Lua 会释放掉相关的内存。
lua_next
int lua_next (lua_State *L, int index);
从栈上弹出一个 key（键），然后把索引指定的表中 key-value（健值）对压入堆栈（指定 key 后面的下一 (next) 对）。如果表中以无更多元素，那么lua_next 将返回 0 （什么也不压入堆栈）。
典型的遍历方法是这样的：
     /* table 放在索引 't' 处 */
     lua_pushnil(L);  /* 第一个 key */
     while (lua_next(L, t) != 0) {
       /* 用一下 'key' （在索引 -2 处） 和 'value' （在索引 -1 处） */
       printf("%s - %s\n",
              lua_typename(L, lua_type(L, -2)),
              lua_typename(L, lua_type(L, -1)));
       /* 移除 'value' ；保留 'key' 做下一次迭代 */
       lua_pop(L, 1);
     }
在遍历一张表的时候，不要直接对 key 调用 lua_tolstring ，除非你知道这个 key 一定是一个字符串。调用 lua_tolstring 有可能改变给定索引位置的值；这会对下一次调用 lua_next 造成影响。
lua_Number
typedef double lua_Number;
Lua 中数字的类型。确省是 double ，但是你可以在 luaconf.h 中修改它。
通过修改配置文件你可以改变 Lua 让它操作其它数字类型（例如：float 或是 long ）。
lua_objlen
size_t lua_objlen (lua_State *L, int index);
返回指定的索引处的值的长度。对于 string ，那就是字符串的长度；对于 table ，是取长度操作符 ('#') 的结果；对于 userdata ，就是为其分配的内存块的尺寸；对于其它值，为 0 。
lua_pcall
lua_pcall (lua_State *L, int nargs, int nresults, int errfunc);
以保护模式调用一个函数。
nargs 和 nresults 的含义与 lua_call 中的相同。如果在调用过程中没有发生错误， lua_pcall 的行为和 lua_call 完全一致。但是，如果有错误发生的话， lua_pcall 会捕获它，然后把单一的值（错误信息）压入堆栈，然后返回错误码。同 lua_call 一样， lua_pcall 总是把函数本身和它的参数从栈上移除。
如果 errfunc 是 0 ，返回在栈顶的错误信息就和原始错误信息完全一致。否则，errfunc 就被当成是错误处理函数在栈上的索引。（在当前的实现里，这个索引不能是伪索引。）在发生运行时错误时，这个函数会被调用而参数就是错误信息。错误处理函数的返回值将被lua_pcall 作为出错信息返回在堆栈上。
典型的用法中，错误处理函数被用来在出错信息上加上更多的调试信息，比如栈跟踪信息 (stack traceback) 。这些信息在lua_pcall 返回后，因为栈已经展开 (unwound) ，所以收集不到了。
lua_pcall 函数在调用成功时返回 0 ，否则返回以下（定义在lua.h 中的）错误代码中的一个：
LUA_ERRRUN: 运行时错误。
LUA_ERRMEM: 内存分配错误。 对于这种错，Lua 调用不了错误处理函数。
LUA_ERRERR: 在运行错误处理函数时发生的错误。
lua_pop
void lua_pop (lua_State *L, int n);
从堆栈中弹出 n 个元素。
lua_pushboolean
void lua_pushboolean (lua_State *L, int b);
把 b 作为一个 boolean 值压入堆栈。
lua_pushcclosure
void lua_pushcclosure (lua_State *L, lua_CFunction fn, int n);
把一个新的 C closure 压入堆栈。
当创建了一个 C 函数后，你可以给它关联一些值，这样就是在创建一个 C closure （参见 §3.4）；接下来无论函数何时被调用，这些值都可以被这个函数访问到。为了将一些值关联到一个 C 函数上，首先这些值需要先被压入堆栈（如果有多个值，第一个先压）。接下来调用lua_pushcclosure 来创建出 closure 并把这个 C 函数压到堆栈上。参数n 告之函数有多少个值需要关联到函数上。lua_pushcclosure 也会把这些值从栈上弹出。
lua_pushcfunction
void lua_pushcfunction (lua_State *L, lua_CFunction f);
将一个 C 函数压入堆栈。这个函数接收一个 C 函数指针，并将一个类型为 function 的 Lua 值压入堆栈。当这个栈顶的值被调用时，将触发对应的 C 函数。
注册到 Lua 中的任何函数都必须遵循正确的协议来接收参数和返回值（参见 lua_CFunction）。
lua_pushcfunction 是作为一个宏定义出现的：
     #define lua_pushcfunction(L,f)  lua_pushcclosure(L,f,0)
lua_pushfstring
const char *lua_pushfstring (lua_State *L, const char *fmt, ...);
把一个格式化过的字符串压入堆栈，然后返回这个字符串的指针。它和 C 函数 sprintf 比较像，不过有一些重要的区别：
摸你需要为结果分配空间： 其结果是一个 Lua 字符串，由 Lua 来关心其内存分配 （同时通过垃圾收集来释放内存）。
这个转换非常的受限。 不支持 flag ，宽度，或是指定精度。 它只支持下面这些： '%%' （插入一个 '%'）， '%s' （插入一个带零终止符的字符串，没有长度限制）， '%f' （插入一个lua_Number）， '%p' （插入一个指针或是一个十六进制数）， '%d' （插入一个int)， '%c' （把一个int 作为一个字符插入）。
lua_pushinteger
void lua_pushinteger (lua_State *L, lua_Integer n);
把 n 作为一个数字压栈。
lua_pushlightuserdata
void lua_pushlightuserdata (lua_State *L, void *p);
把一个 light userdata 压栈。
userdata 在 Lua 中表示一个 C 值。 light userdata 表示一个指针。它是一个像数字一样的值：你不需要专门创建它，它也没有独立的 metatable ，而且也不会被收集（因为从来不需要创建）。只要表示的 C 地址相同，两个 light userdata 就相等。
lua_pushlstring
void lua_pushlstring (lua_State *L, const char *s, size_t len);
把指针 s 指向的长度为 len 的字符串压栈。 Lua 对这个字符串做一次内存拷贝（或是复用一个拷贝），因此s 处的内存在函数返回后，可以释放掉或是重用于其它用途。字符串内可以保存有零字符。
lua_pushnil
void lua_pushnil (lua_State *L);
把一个 nil 压栈。
lua_pushnumber
void lua_pushnumber (lua_State *L, lua_Number n);
把一个数字 n 压栈。
lua_pushstring
void lua_pushstring (lua_State *L, const char *s);
把指针 s 指向的以零结尾的字符串压栈。 Lua 对这个字符串做一次内存拷贝（或是复用一个拷贝），因此s 处的内存在函数返回后，可以释放掉或是重用于其它用途。字符串中不能包含有零字符；第一个碰到的零字符会认为是字符串的结束。
lua_pushthread
int lua_pushthread (lua_State *L);
把 L 中提供的线程压栈。如果这个线程是当前状态机的主线程的话，返回 1 。
lua_pushvalue
void lua_pushvalue (lua_State *L, int index);
把堆栈上给定有效处索引处的元素作一个拷贝压栈。
lua_pushvfstring
const char *lua_pushvfstring (lua_State *L,
                              const char *fmt,
                              va_list argp);
等价于 lua_pushfstring，不过是用 va_list 接收参数，而不是用可变数量的实际参数。
lua_rawequal
int lua_rawequal (lua_State *L, int index1, int index2);
如果两个索引 index1 和 index2 处的值简单地相等（不调用元方法）则返回 1 。否则返回 0 。如果任何一个索引无效也返回 0 。
lua_rawget
void lua_rawget (lua_State *L, int index);
类似于 lua_gettable，但是作一次直接访问（不触发元方法）。
lua_rawgeti
void lua_rawgeti (lua_State *L, int index, int n);
把 t[n] 的值压栈，这里的 t 是指给定索引 index 处的一个值。这是一个直接访问；就是说，它不会触发元方法。
lua_rawset
void lua_rawset (lua_State *L, int index);
类似于 lua_settable，但是是作一个直接赋值（不触发元方法）。
lua_rawseti
void lua_rawseti (lua_State *L, int index, int n);
等价于 t[n] = v，这里的 t 是指给定索引 index 处的一个值，而 v 是栈顶的值。
函数将把这个值弹出栈。赋值操作是直接的；就是说，不会触发元方法。
lua_Reader
typedef const char * (*lua_Reader) (lua_State *L,
                                    void *data,
                                    size_t *size);
lua_load 用到的读取器函数，每次它需要一块新的 chunk 的时候，lua_load 就调用读取器，每次都会传入一个参数data 。读取器需要返回含有新的 chunk 的一块内存的指针，并把 size 设为这块内存的大小。内存块必须在下一次函数被调用之前一直存在。读取器可以通过返回一个NULL 来指示 chunk 结束。读取器可能返回多个块，每个块可以有任意的大于零的尺寸。
lua_register
void lua_register (lua_State *L,
                   const char *name,
                   lua_CFunction f);
把 C 函数 f 设到全局变量 name 中。它通过一个宏定义：
     #define lua_register(L,n,f) \
            (lua_pushcfunction(L, f), lua_setglobal(L, n))
lua_remove
void lua_remove (lua_State *L, int index);
从给定有效索引处移除一个元素，把这个索引之上的所有元素移下来填补上这个空隙。不能用伪索引来调用这个函数，因为伪索引并不指向真实的栈上的位置。
lua_replace
void lua_replace (lua_State *L, int index);
把栈顶元素移动到给定位置（并且把这个栈顶元素弹出），不移动任何元素（因此在那个位置处的值被覆盖掉）。
lua_resume
int lua_resume (lua_State *L, int narg);
在给定线程中启动或继续一个 coroutine 。
要启动一个 coroutine 的话，首先你要创建一个新线程（参见 lua_newthread ）；然后把主函数和若干参数压到新线程的堆栈上；最后调用 lua_resume ，把 narg 设为参数的个数。这次调用会在 coroutine 挂起时或是结束运行后返回。当函数返回时，堆栈中会有传给lua_yield 的所有值，或是主函数的所有返回值。如果 coroutine 切换时，lua_resume 返回LUA_YIELD ，而当 coroutine 结束运行且没有任何错误时，返回 0 。如果有错则返回错误代码（参见lua_pcall）。在发生错误的情况下，堆栈没有展开，因此你可以使用 debug API 来处理它。出错信息放在栈顶。要继续运行一个 coroutine 的话，你把需要传给yield 作结果的返回值压入堆栈，然后调用lua_resume 。
lua_setallocf
void lua_setallocf (lua_State *L, lua_Alloc f, void *ud);
把指定状态机的分配器函数换成带上指针 ud 的 f 。
lua_setfenv
int lua_setfenv (lua_State *L, int index);
从堆栈上弹出一个 table 并把它设为指定索引处值的新环境。如果指定索引处的值即不是函数又不是线程或是 userdata ，lua_setfenv 会返回 0 ，否则返回 1 。
lua_setfield
void lua_setfield (lua_State *L, int index, const char *k);
做一个等价于 t[k] = v 的操作，这里 t 是给出的有效索引index 处的值，而v 是栈顶的那个值。
这个函数将把这个值弹出堆栈。跟在 Lua 中一样，这个函数可能触发一个 "newindex" 事件的元方法（参见§2.8）。
lua_setglobal
void lua_setglobal (lua_State *L, const char *name);
从堆栈上弹出一个值，并将其设到全局变量 name 中。它由一个宏定义出来：
     #define lua_setglobal(L,s)   lua_setfield(L, LUA_GLOBALSINDEX, s)
lua_setmetatable
int lua_setmetatable (lua_State *L, int index);
把一个 table 弹出堆栈，并将其设为给定索引处的值的 metatable 。
lua_settable
void lua_settable (lua_State *L, int index);
作一个等价于 t[k] = v 的操作，这里 t 是一个给定有效索引index 处的值，v 指栈顶的值，而 k 是栈顶之下的那个值。
这个函数会把键和值都从堆栈中弹出。和在 Lua 中一样，这个函数可能触发 "newindex" 事件的元方法（参见§2.8）。
lua_settop
void lua_settop (lua_State *L, int index);
参数允许传入任何可接受的索引以及 0 。它将把堆栈的栈顶设为这个索引。如果新的栈顶比原来的大，超出部分的新元素将被填为nil 。如果index 为 0 ，把栈上所有元素移除。
lua_State
typedef struct lua_State lua_State;
一个不透明的结构，它保存了整个 Lua 解释器的状态。 Lua 库是完全可重入的：它没有任何全局变量。（译注：从 C 语法上来说，也不尽然。例如，在 table 的实现中用了一个静态全局变量 dummynode_ ，但这在正确使用时并不影响可重入性。只是万一你错误链接了 lua 库，不小心在同一进程空间中存在两份 lua 库实现的代码的话，多份 dummynode_ 不同的地址会导致一些问题。）所有的信息都保存在这个结构中。
这个状态机的指针必须作为第一个参数传递给每一个库函数。 lua_newstate 是一个例外，这个函数会从头创建一个 Lua 状态机。
lua_status
int lua_status (lua_State *L);
返回线程 L 的状态。
正常的线程状态是 0 。当线程执行完毕或发生一个错误时，状态值是错误码。如果线程被挂起，状态为 LUA_YIELD 。
lua_toboolean
int lua_toboolean (lua_State *L, int index);
把指定的索引处的的 Lua 值转换为一个 C 中的 boolean 值（ 0 或是 1 ）。和 Lua 中做的所有测试一样，lua_toboolean 会把任何不同于false 和nil 的值当作 1 返回；否则就返回 0 。如果用一个无效索引去调用也会返回 0 。（如果你想只接收真正的 boolean 值，就需要使用lua_isboolean 来测试值的类型。）
lua_tocfunction
lua_CFunction lua_tocfunction (lua_State *L, int index);
把给定索引处的 Lua 值转换为一个 C 函数。这个值必须是一个 C 函数；如果不是就返回 NULL 。
lua_tointeger
lua_Integer lua_tointeger (lua_State *L, int idx);
把给定索引处的 Lua 值转换为 lua_Integer 这样一个有符号整数类型。这个 Lua 值必须是一个数字或是一个可以转换为数字的字符串（参见 §2.2.1）；否则，lua_tointeger 返回 0 。
如果数字不是一个整数，截断小数部分的方式没有被明确定义。
lua_tolstring
const char *lua_tolstring (lua_State *L, int index, size_t *len);
把给定索引处的 Lua 值转换为一个 C 字符串。如果 len 不为 NULL ，它还把字符串长度设到*len 中。这个 Lua 值必须是一个字符串或是一个数字；否则返回返回NULL 。如果值是一个数字，lua_tolstring 还会把堆栈中的那个值的实际类型转换为一个字符串。（当遍历一个表的时候，把lua_tolstring 作用在键上，这个转换有可能导致lua_next 弄错。）
lua_tolstring 返回 Lua 状态机中字符串的以对齐指针。这个字符串总能保证 （ C 要求的）最后一个字符为零 ('\0') ，而且它允许在字符串内包含多个这样的零。因为 Lua 中可能发生垃圾收集，所以不保证lua_tolstring 返回的指针，在对应的值从堆栈中移除后依然有效。
lua_tonumber
lua_Number lua_tonumber (lua_State *L, int index);
把给定索引处的 Lua 值转换为 lua_Number 这样一个 C 类型（参见 lua_Number ）。这个 Lua 值必须是一个数字或是一个可转换为数字的字符串（参见 §2.2.1 ）；否则，lua_tonumber 返回 0 。
lua_topointer
const void *lua_topointer (lua_State *L, int index);
把给定索引处的值转换为一般的 C 指针 (void*) 。这个值可以是一个 userdata ，table ，thread 或是一个 function ；否则，lua_topointer 返回NULL 。不同的对象有不同的指针。不存在把指针再转回原有类型的方法。
这个函数通常只为产生 debug 信息用。
lua_tostring
const char *lua_tostring (lua_State *L, int index);
等价于 lua_tolstring ，而参数 len 设为 NULL 。
lua_tothread
lua_State *lua_tothread (lua_State *L, int index);
把给定索引处的值转换为一个 Lua 线程（由 lua_State* 代表）。这个值必须是一个线程；否则函数返回NULL 。
lua_touserdata
void *lua_touserdata (lua_State *L, int index);
如果给定索引处的值是一个完整的 userdata ，函数返回内存块的地址。如果值是一个 light userdata ，那么就返回它表示的指针。否则，返回NULL 。
lua_type
int lua_type (lua_State *L, int index);
返回给定索引处的值的类型，当索引无效时则返回 LUA_TNONE （那是指一个指向堆栈上的空位置的索引）。lua_type 返回的类型是一些个在lua.h 中定义的常量：LUA_TNIL ， LUA_TNUMBER ， LUA_TBOOLEAN ， LUA_TSTRING ， LUA_TTABLE ， LUA_TFUNCTION ， LUA_TUSERDATA ， LUA_TTHREAD ， LUA_TLIGHTUSERDATA 。
lua_typename
const char *lua_typename  (lua_State *L, int tp);
返回 tp 表示的类型名，这个 tp 必须是 lua_type 可能返回的值中之一。
lua_Writer
typedef int (*lua_Writer) (lua_State *L,
                           const void* p,
                           size_t sz,
                           void* ud);
由 lua_dump 用到的写入器函数。每次 lua_dump 产生了一块新的 chunk ，它都会调用写入器。传入要写入的缓存 (p) 和它的尺寸 (sz) ，还有lua_dump 的参数data 。
写入器会返回一个错误码： 0 表示没有错误；别的值均表示一个错误，并且会让 lua_dump 停止再次调用写入器。
lua_xmove
void lua_xmove (lua_State *from, lua_State *to, int n);
传递 同一个 全局状态机下不同线程中的值。
这个函数会从 from 的堆栈中弹出 n 个值，然后把它们压入to 的堆栈中。
lua_yield
int lua_yield  (lua_State *L, int nresults);
切出一个 coroutine 。
这个函数只能在一个 C 函数的返回表达式中调用。如下：
     return lua_yield (L, nresults);
当一个 C 函数这样调用 lua_yield ，正在运行中的 coroutine 将从运行中挂起，然后启动这个 coroutine 用的那次对 lua_resume 的调用就返回了。参数 nresults 指的是堆栈中需要返回的结果个数，这些返回值将被传递给lua_resume 。
3.8 - 调试接口
Lua 没有内建的调试设施。取而代之的是提供了一些函数接口和钩子。利用这些接口，可以做出一些不同类型的调试器，性能分析器，或是其它一些需要从解释器中取到“内部信息”的工具。
lua_Debug
typedef struct lua_Debug {
  int event;
  const char *name;           /* (n) */
  const char *namewhat;       /* (n) */
  const char *what;           /* (S) */
  const char *source;         /* (S) */
  int currentline;            /* (l) */
  int nups;                   /* (u) upvalue 个数 */
  int linedefined;            /* (S) */
  int lastlinedefined;        /* (S) */
  char short_src[LUA_IDSIZE]; /* (S) */
  /* 私有部分 */
  其它域
} lua_Debug;
一个用来携带活动中函数的各种信息的结构。 lua_getstack 仅填写这个结构中的私有部分，这些部分以后会用到。调用 lua_getinfo 则可以填上 lua_Debug 中有用信息的那些域。
lua_Debug 中的各个域有下列含义：
source: 如果函数是定义在一个字符串中，source 就是这个字符串。 如果函数定义在一个文件中，source 是一个以 '@' 开头的文件名。
short_src: 一个“可打印版本”的 source，用于出错信息。
linedefined: 函数定义开始处的行号。
lastlinedefined: 函数定义结束处的行号。
what: 如果函数是一个 Lua 函数，则为一个字符串 "Lua" ； 如果是一个 C 函数，则为"C"； 如果它是一个 chunk 的主体部分，则为"main"； 如果是一个作了尾调用的函数，则为 "tail" 。 别的情况下，Lua 没有关于函数的别的信息。
currentline: 给定函数正在执行的那一行。 当提供不了行号信息的时候，currentline 被设为 -1 。
name: 给定函数的一个合理的名字。 因为 Lua 中的函数也是一个值， 所以它们没有固定的名字： 一些函数可能是全局复合变量的值， 另一些可能仅仅只是被保存在一个 table 中。lua_getinfo 函数会检查函数是这样被调用的，以此来找到一个适合的名字。 如果它找不到名字，name 就被设置为NULL 。
namewhat: 结实 name 域。 namewhat 的值可以是"global","local", "method", "field","upvalue", 或是"" （空串）。 这取决于函数怎样被调用。 （Lua 用空串表示其它选项都不符合）
nups: 函数的 upvalue 的个数。
lua_gethook
lua_Hook lua_gethook (lua_State *L);
返回当前的钩子函数。
lua_gethookcount
int lua_gethookcount (lua_State *L);
返回当前钩子记数。
lua_gethookmask
int lua_gethookmask (lua_State *L);
返回当前的钩子掩码 (mask) 。
lua_getinfo
int lua_getinfo (lua_State *L, const char *what, lua_Debug *ar);
返回一个指定的函数或函数调用的信息。
当用于取得一次函数调用的信息时，参数 ar 必须是一个有效的活动的记录。这条记录可以是前一次调用lua_getstack 得到的，或是一个钩子 （参见lua_Hook）得到的参数。
用于获取一个函数的信息时，可以把这个函数压入堆栈，然后把 what 字符串以字符 '>' 起头。（这个情况下，lua_getinfo 从栈顶上弹出函数。）例如，想知道函数f 在哪一行定义的，你可以下下列代码：
     lua_Debug ar;
     lua_getfield(L, LUA_GLOBALSINDEX, "f");  /* 取到全局变量 'f' */
     lua_getinfo(L, ">S", &ar);
     printf("%d\n", ar.linedefined);
what 字符串中的每个字符都筛选出结构 ar 结构中一些域用于填充，或是把一个值压入堆栈：
'n': 填充 name 及 namewhat 域；
'S': 填充 source， short_src，linedefined，lastlinedefined，以及 what 域；
'l': 填充 currentline 域；
'u': 填充 nups 域；
'f': 把正在运行中指定级别处函数压入堆栈； （译注：一般用于获取函数调用中的信息， 级别是由 ar 中的私有部分来提供。 如果用于获取静态函数，那么就直接把指定函数重新压回堆栈， 但这样做通常无甚意义。）
'L': 压一个 table 入栈，这个 table 中的整数索引用于描述函数中哪些行是有效行。 （有效行指有实际代码的行， 即你可以置入断点的行。 无效行包括空行和只有注释的行。）
这个函数出错会返回 0 （例如，what 中有一个无效选项）。
lua_getlocal
const char *lua_getlocal (lua_State *L, lua_Debug *ar, int n);
从给定活动记录中获取一个局部变量的信息。参数 ar 必须是一个有效的活动的记录。这条记录可以是前一次调用lua_getstack 得到的，或是一个钩子 （参见lua_Hook）得到的参数。索引n 用于选择要检阅哪个局部变量（ 1 表示第一个参数或是激活的第一个局部变量，以此类推，直到最后一个局部变量）。 lua_getlocal 把变量的值压入堆栈并返回它的名字。
以 '(' （正小括号）开始的变量指内部变量（循环控制变量，临时变量，C 函数局部变量）。
当索引大于局部变量的个数时，返回 NULL （什么也不压入）。
lua_getstack
int lua_getstack (lua_State *L, int level, lua_Debug *ar);
获取解释器的运行时栈的信息。
这个函数用正在运行中的给定级别处的函数的活动记录来填写 lua_Debug 结构的一部分。 0 级表示当前运行的函数，而 n+1 级处的函数就是调用第 n 级函数的那一个。如果没有错误，lua_getstack 返回 1 ；当调用传入的级别大于堆栈深度的时候，返回 0 。
lua_getupvalue
const char *lua_getupvalue (lua_State *L, int funcindex, int n);
获取一个 closure 的 upvalue 信息。（对于 Lua 函数，upvalue 是函数需要使用的外部局部变量，因此这些变量被包含在 closure 中。）lua_getupvalue 获取第n 个 upvalue ，把这个 upvalue 的值压入堆栈，并且返回它的名字。 funcindex 指向堆栈上 closure 的位置。（ 因为 upvalue 在整个函数中都有效，所以它们没有特别的次序。因此，它们以字母次序来编号。）
当索引号比 upvalue 数量大的时候，返回 NULL （而且不会压入任何东西）对于 C 函数，这个函数用空串"" 表示所有 upvalue 的名字。
lua_Hook
typedef void (*lua_Hook) (lua_State *L, lua_Debug *ar);
用于调试的钩子函数类型。
无论何时钩子被调用，它的参数 ar 中的 event 域都被设为触发钩子的事件。 Lua 把这些事件定义为以下常量：LUA_HOOKCALL，LUA_HOOKRET, LUA_HOOKTAILRET，LUA_HOOKLINE， andLUA_HOOKCOUNT。除此之外，对于 line 事件，currentline 域也被设置。要想获得ar 中的其他域，钩子必须调用lua_getinfo。对于返回事件，event 的正常值可能是LUA_HOOKRET，或者是LUA_HOOKTAILRET 。对于后一种情况，Lua 会对一个函数做的尾调用也模拟出一个返回事件出来；对于这个模拟的返回事件，调用lua_getinfo 没有什么作用。
当 Lua 运行在一个钩子内部时，它将屏蔽掉其它对钩子的调用。也就是说，如果一个钩子函数内再调回 Lua 来执行一个函数或是一个 chunk ，这个执行操作不会触发任何的钩子。
lua_sethook
int lua_sethook (lua_State *L, lua_Hook f, int mask, int count);
设置一个调试用钩子函数。
参数 f 是钩子函数。 mask 指定在哪些事件时会调用：它由下列一组位常量构成LUA_MASKCALL，LUA_MASKRET， LUA_MASKLINE，以及LUA_MASKCOUNT。参数count 只在 mask 中包含有LUA_MASKCOUNT 才有意义。对于每个事件，钩子被调用的情况解释如下：
call hook: 在解释器调用一个函数时被调用。 钩子将于 Lua 进入一个新函数后，函数获取参数前被调用。
return hook: 在解释器从一个函数中返回时调用。 钩子将于 Lua 离开函数之前的那一刻被调用。 你无权访问被函数返回出去的那些值。（译注：原文 (You have no access to the values to be returned by the function) 如此。 但“无权访问”一词值得商榷。 某些情况下你可以访问到一些被命名为 (*temporary) 的局部变量， 那些索引被排在最后的 (*temporary) 变量指的就是返回值。 但是由于 Lua 对特殊情况做了一些优化，比如直接返回一个被命名的局部变量， 那么就找不到对应的 (*temporary) 变量了。本质上，返回值一定存在于此刻的局部变量中， 并且可以访问它，只是无法确定是哪些罢了。至于这个时候函数体内的其它局部变量， 是不保证有效的。进入 return hook 的那一刻起，实际已经退出函数内部的运行环节， 返回值占用的局部变量空间以后的部分，都有可能因 hook 本身复用它们而改变。）
line hook: 在解释器准备开始执行新的一行代码时， 或是跳转到这行代码中时（即使在同一行内跳转）被调用。 （这个事件仅仅在 Lua 执行一个 Lua 函数时发生。）
count hook: 在解释器每执行 count 条指令后被调用。 （这个事件仅仅在 Lua 执行一个 Lua 函数时发生。）
钩子可以通过设置 mask 为零屏蔽。
lua_setlocal
const char *lua_setlocal (lua_State *L, lua_Debug *ar, int n);
设置给定活动记录中的局部变量的值。参数 ar 与 n 和 lua_getlocal 中的一样（参见 lua_getlocal）。 lua_setlocal 把栈顶的值赋给变量然后返回变量的名字。它会将值从栈顶弹出。
当索引大于局部变量的个数时，返回 NULL （什么也不弹出）。
 
lua_setupvalue
const char *lua_setupvalue (lua_State *L, int funcindex, int n);
设置 closure 的 upvalue 的值。它把栈顶的值弹出并赋于 upvalue 并返回 upvalue 的名字。参数funcindex 与 n 和 lua_getupvalue 中的一样（参见 lua_getupvalue）。
当索引大于 upvalue 的个数时，返回 NULL （什么也不弹出）。