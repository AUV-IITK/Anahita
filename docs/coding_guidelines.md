Use [ usual steps](https://akrabat.com/the-beginners-guide-to-contributing-to-a-github-project/) to make changes into the project just like other GitHub projects.

# Code style
* _Cpp_ follow ROS [Cpp Style Guide](http://wiki.ros.org/CppStyleGuide) rigorously.  
* _CMake_ follow Caktin [CMake Style Guide](http://docs.ros.org/jade/api/catkin/html/user_guide/standards.html).  
* _Python_ code should follow [pep8](http://www.python.org/dev/peps/pep-0008/). Please see [ROS PyStyleGuide](http://wiki.ros.org/PyStyleGuide) for details. Use `Anaconda` plugin for sublime text editor for style check.  
* _Bash_ Use this [python script](http://arachnoid.com/python/beautify_bash_program.html).  
* _Roslint_
All pkgs (except `commons` pkgs which don't have cpp or py code) depend on [roslint](http://wiki.ros.org/roslint) for static code analysis.  

__Note:__ [Utils](https://github.com/AUV-IITK/auv2017/tree/master/utils) contains scripts for formatting `cpp`, `python`, `xml`, `CMakeLists` files. Since these are bash scripts, there is a formatting script for `bash`.

***

# Documentation

We maintain two forms of online documentation for ROS packages:  
* _wiki pages_ / External docs: This is the AUVWiki repo. Home for tutorials, design documents and other sorts of information. This is mainly for people running your code. In general anything that people need to know outside source i.e. while only running auv code goes in the auv-wiki, anything else goes to package doc. 

* _auto-generated code documentation_ Internal docs: This is mainly for people integrating with or editing your code. There are auto generated from source code comments using doxygen. Travis CI generated and auto deploys to gh-pages branch. Located at http://auv-iitk.github.io/auv/ . Look at [sample commit](https://github.com/AUV-IITK/auv/pull/103/commits/c23d7efdadbfd611276e20509f98e6b39328fa20) to see how to add documentation. Also [Doxygen manual](https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html) is good for quick start.

***

# Source control
We use Git. Read the wiki page on git.

***

# Bug tracking
We use github issues on auv repo for tracking bugs.

***
# (_Still missing_)  Testing
We use two level of testing:  
* Library: At the library level, we use standard unit-test frameworks. In C++, we use [gtest](http://wiki.ros.org/gtest). In Python, we use [unittest](http://wiki.ros.org/unittest).  
* Message: At the message level, we use [rostest](http://wiki.ros.org/rostest) to set up a system of ROS nodes, run a test node, then tear down the system. 
 
ROS has established best [practices and policies](http://wiki.ros.org/UnitTesting) for writing and running tests.  
Auv code is structured in layers, with each layer interacting with the other using `actionlib`. For testing each layer separately we use unit tests nodes which are part of the concerning layer itself.  

***

# Standardization
Code should use ROS services, follow guidelines for their use:  
use [rosout](http://wiki.ros.org/rosout) for printing messages  
uses the ROS [Clock](http://wiki.ros.org/Clock) for time-based routines

***

# Large data files
Large files (anything over 1MB, really) often don't belong in the auv repositories, especially if they are just used for tests. These large files affect the time that it takes to checkout the repository.

***

#  Naming Convention
Please refer to [ROS CppStyleGuide](http://wiki.ros.org/CppStyleGuide#Naming) and [ROS Conventions](http://wiki.ros.org/ROS/Patterns/Conventions#Naming_ROS_Resources).

Avoid using any sort of Hungarian notation on names and "_ptr" on pointers.

| **Code Element** | **Style** | **Comment** |
| --- | --- | --- |
| Namespace | under\_scored | Differentiate from class names |
| Class name | CamelCase | To differentiate from STL types which ISO recommends (do not use "C" or "T" prefixes) |
| Function name | camelCase | Lower case start is almost universal except for .Net world |
| Parameters/Locals | under\_scored | Vast majority of standards recommends this because \_ is more readable to C++ crowd (although not much to Java/.Net crowd) |
| Member variables | under\_scored\_with\_ | The prefix \_ is heavily discouraged as ISO has rules around reserving \_identifiers, so we recommend suffix instead |
| Enums and its members | CamelCase | Most except very old standards agree with this one |
| Globals | g\_under\_scored | You shouldn't have these in first place! |
| Constants | UPPER\_CASE | Very contentious and we just have to pick one here, unless if is a private constant in class or method, then use naming for Members or Locals |
| File names | Match case of class name in file | Lot of pro and cons either way but this removes inconsistency in auto generated code (important for ROS) |

## Header Files

Use a namespace qualified #ifdef to protect against multiple inclusion:

```
#ifndef msr_airsim_MyHeader_hpp
#define msr_airsim_MyHeader_hpp

//--your code

#endif
```

The reason we don't use #pragma once is because it's not supported if same header file exists at multiple places (which might be possible under ROS build system!).

## Bracketing

Inside function or method body place curly bracket on same line. 
Outside that the Namespace, Class and methods levels use separate line.
This is called [K&amp;R style](https://en.wikipedia.org/wiki/Indent_style#K.26R_style) and its variants are widely used in C++ vs other styles which are more popular in other languages. 
Notice that curlies are not required if you have single statement, but complex statements are easier to keep correct with the braces.

```
int main(int argc, char* argv[])
{
     while (x == y) {
        f0();
        if (cont()) {
            f1();
        } else {
            f2();
            f3();
        }
        if (x > 100)
            break;
    }
}
```

## Const and References

Religiously review all non-scalar parameters you declare to be candidate for const and references. If you are coming from languages such as C#/Java/Python,
the most often mistake you would make is to pass parameters by value instead of `const T&;` Especially most of the strings, vectors and maps you want to 
pass as `const T&;` (if they are readonly) or `T&` (if they are writable). Also add `const` suffix to methods as much as possible.

## Overriding
When overriding virtual method, use override suffix.


## Pointers

This is really about memory management.  A simulator has much performance cricial code, so we try and avoid overloading the memory manager
with lots of calls to new/delete.  We also want to avoid too much copying of things on the stack, so we pass things by reference when ever possible.
But when the object really needs to live longer than the call stack you often need to allocate that object on
the heap, and so you have a pointer.  Now, if management of the lifetime of that object is going to be tricky we recommend using 
[C++ 11 smart pointers](https://cppstyle.wordpress.com/c11-smart-pointers/). 
But smart pointers do have a cost, so don’t use them blindly everywhere.  For private code 
where performance is paramount, raw pointers can be used.  Raw pointers are also often needed when interfacing with legacy systems
that only accept pointer types, for example, sockets API.  But we try to wrap those legacy interfaces as
much as possible and avoid that style of programming from leaking into the larger code base.  

Religiously check if you can use const everywhere, for example, `const float * const xP`. Avoid using prefix or suffix to indicate pointer types in variable names, i.e. use `my_obj` instead of `myobj_ptr` except in cases where it might make sense to differentiate variables better, for example, `int mynum = 5; int* mynum_ptr = mynum;`

## ROS Naming Convention  

* Topics and nodes should be names keeping in mind the `ros namespace`. For keeping it noob friendly, global names should be assigned to nodes and topics.  
* Package folder name should be same as package name.  
* Node file name, executable name and ros node name should be same.  
* Dynamic Reconfigure cfg files should be related to what it helps configuring.  
* Action files should be named after what action or task is done through them.  
* Launch files should be named after what state it will launch the nodes in.  

***
