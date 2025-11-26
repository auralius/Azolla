# Azolla

Azolla is a C++ framework for simulating mobile robots. It is designed with a strong focus on probabilistic robotics, and provides core features such as:

- Odometry  
- Occupancy grid algorithms  
- Mapping  
- Localization  

The main goal of Azolla is to offer a simple 2D mobile robot simulator that can still be used to test sophisticated algorithms. More specifically, Azolla aims to be:

- Lightweight  
- Easy to use  
- Multi-platform  

For detailed usage instructions, refer to:

[Using Azolla.pdf](https://github.com/auralius/Azolla/blob/master/Using%20Azolla.pdf)

---

## Installation

### Linux

#### Using Code::Blocks

1. Install the **FLTK** libraries.
2. Install the **Boost** libraries.
3. Ensure **Code::Blocks** is installed.
4. Open the `build/codeblocks` folder.
5. Open `Azolla.cbp` using Code::Blocks.

#### Using Eclipse

You can also build and run Azolla using **Eclipse**:

1. In Eclipse, select **“Import existing project into workspace”**.
2. Navigate to the `src` folder.
3. Select the `.cproject` and `.project` files to import the project.

#### Using Makefile (no IDE)

A `Makefile` is provided in the `src` folder if you prefer not to use an IDE.  

The compiled binary will be generated in the `src/bin` directory.

---

### Windows

A Visual Studio project file is provided for Windows users.  
However, it has not been updated since the last modification, so it may no longer work as-is.
