# common-lisp-jupyter

* Install cram to you workspace by following [this guide](https://cram-system.org/installation)
* Verify your installation by running `roslisp_repl` and loading a tutorial, as mentioned in CRAM's general README
* Follow [this guide](https://cram-system.org/tutorials/advanced/jupyter) to install the Python dependencies for the common-lisp-jupyter kernel
  * it will use the rosinstall file in this directory to load the necessary lisp projects into your workspace
* When your workspace is set up with the jupyter deps, install the kernel via sbcl by loading the installer:

```bash
/usr/bin/sbcl --dynamic-space-size 4096 --load sbcl-jupyter-kernel-installer.lisp
```

* Check if the kernel is placed at the right spot. Go to `~/.local/share/jupyter/` and look for `common-lisp`, there should be a `kernel.json`.
* Verify that common-lisp-jupyter works by running `jupyter-lab`. It should load the common-lisp kernel.