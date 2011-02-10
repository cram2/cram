
CL_UTILS_ARCHIVE=cl-utilities-1.2.4.tar.gz
CL_UTILS_URL=http://code.in.tum.de/files/$(CL_UTILS_ARCHIVE)

all: installed

downloaded: Makefile
	make clean
	wget $(CL_UTILS_URL)
	touch downloaded

installed: downloaded
	tar xvzf $(CL_UTILS_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../cl-utilities-1.2.4/cl-utilities.asd asdf/cl-utilities.asd
	touch installed

clean:
	rm -rf cl-utilities-1.2.4 asdf
	rm -f $(CL_UTILS_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed downloaded
