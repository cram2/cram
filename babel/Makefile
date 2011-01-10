
all: installed

downloaded: Makefile
	make clean
	darcs get http://common-lisp.net/project/babel/darcs/babel
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	ln -s ../babel/babel.asd asdf/babel.asd
	ln -s ../babel/babel-streams.asd asdf/babel-streams.asd
	touch installed

clean:
	rm -rf babel asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

