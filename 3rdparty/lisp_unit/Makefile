
all: installed

downloaded: Makefile
	make clean
	git clone https://github.com/OdonataResearchLLC/lisp-unit.git
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../lisp-unit/lisp-unit.asd asdf/
	touch installed

clean:
	rm -rf lisp-unit asdf
	rm -f downloaded rospack_nosubdirs installed
