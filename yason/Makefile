
all: installed

checkedout: Makefile
	make clean
	git clone git://github.com/hanshuebner/yason.git
	touch checkedout

installed: checkedout
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../yason/yason.asd asdf/
	touch installed

clean:
	rm -rf asdf yason rospack_nosubdirs installed checkedout

