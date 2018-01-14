# This target called from top-level makefile to make links before this
# directory is built.  This seems to be required to get the dependency generation to
# work properly.
.PHONY:	links
links: cvmx-helper.c
# Make links to the version of the simple executive that we need to use
cvmx-helper.c: 
	ln -sf ${OCTEON_ROOT}/executive/cvmx*.c .
	ln -sf ${OCTEON_ROOT}/executive/octeon*.c .

