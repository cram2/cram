
all: installed

checkedout: Makefile
	make clean
	git clone git://gitorious.org/trivial-gray-streams/trivial-gray-streams.git
	touch checkedout

installed: checkedout
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../trivial-gray-streams/trivial-gray-streams.asd asdf/
	touch installed

clean:
	rm -rf asdf trivial-gray-streams rospack_nosubdirs installed checkedout

