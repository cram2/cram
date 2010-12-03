
SPLIT_SEQ_ARCHIVE=split-sequence.tar.gz
PACKAGE_DIR=$(shell rospack find split_sequence)

all: installed

downloaded: Makefile
	make clean
	wget http://ftp.linux.org.uk/pub/lisp/experimental/cclan/$(SPLIT_SEQ_ARCHIVE)
	touch downloaded

installed: downloaded
	tar xvzf $(SPLIT_SEQ_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ${PACKAGE_DIR}/split-sequence/split-sequence.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf split-sequence asdf
	rm -f $(SPLIT_SEQ_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed downloaded

