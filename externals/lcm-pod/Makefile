DL_FILE := lcm-1.0.0.tar.gz
DL_LINK := http://lcm.googlecode.com/files/
UNZIP_DIR := lcm-1.0.0

all: $(UNZIP_DIR)/Makefile
	$(MAKE) -C $(UNZIP_DIR) install

# Default to a less-verbose build.  If you want all the gory compiler output,
# run "make VERBOSE=1"
$(VERBOSE).SILENT:

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to four parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX:=$(shell for pfx in .. ../.. ../../.. ../../../..; do d=`pwd`/$$pfx/build;\
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif
# create the build directory if needed, and normalize its path name
BUILD_PREFIX:=$(shell mkdir -p $(BUILD_PREFIX) && cd $(BUILD_PREFIX) && echo `pwd`)

# Default to a release build.  If you want to enable debugging flags, run
# "make BUILD_TYPE=Debug"
OPT_FLAGS := #-g -O2
ifeq "$(BUILD_TYPE)" "Debug"
OPT_FLAGS = -g
endif

$(UNZIP_DIR)/Makefile:
	$(MAKE) configure

.PHONY: configure
configure: $(UNZIP_DIR)/configure
	# run configure
	@cd $(UNZIP_DIR)  && \
		./configure --prefix=$(BUILD_PREFIX) \
		INSTALL="`which install` -c -C" \
		PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(BUILD_PREFIX)/lib/pkgconfig \
		CFLAGS="-I$(BUILD_PREFIX)/include $(OPT_FLAGS) $(CFLAGS)" \
		CXXFLAGS="-I$(BUILD_PREFIX)/include $(OPT_FLAGS) $(CXXFLAGS)" \
		LDFLAGS="-L$(BUILD_PREFIX)/lib $(LDFLAGS)"

$(UNZIP_DIR)/configure:
	@echo "\nDownloading lcm \n\n"
	wget $(DL_LINK)/$(DL_FILE)
	@echo "\nunzipping to $(UNZIP_DIR) \n\n"
	tar -xzvf $(DL_FILE) && rm $(DL_FILE)
	@echo "\nBUILD_PREFIX: $(BUILD_PREFIX)\n\n"

install_prereqs_homebrew :
	brew install wget glib coreutils

install_prereqs_cygwin:
	cygwin-setup -q -P gcc-g++ wget libglib2.0-devel

clean:
	-if [ -e $(UNZIP_DIR)/Makefile ]; then $(MAKE) -C $(UNZIP_DIR) clean uninstall; fi
	-if [ -e $(UNZIP_DIR)/Makefile ]; then $(MAKE) -C $(UNZIP_DIR) distclean; fi

