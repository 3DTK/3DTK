SET(SPARSE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/cxsparse/cxsparse-4.4.4/suitesparse/)
SET(SPARSE_SOURCES
	${SPARSE_DIR}cs_add.c
	${SPARSE_DIR}cs_amd.c
	${SPARSE_DIR}cs_chol.c
	${SPARSE_DIR}cs_cholsol.c
	${SPARSE_DIR}cs_compress.c
	${SPARSE_DIR}cs_counts.c
	${SPARSE_DIR}cs_cumsum.c
	${SPARSE_DIR}cs_dfs.c
	${SPARSE_DIR}cs_dmperm.c
	${SPARSE_DIR}cs_droptol.c
	${SPARSE_DIR}cs_dropzeros.c
	${SPARSE_DIR}cs_dupl.c
	${SPARSE_DIR}cs_entry.c
	${SPARSE_DIR}cs_ereach.c
	${SPARSE_DIR}cs_etree.c
	${SPARSE_DIR}cs_fkeep.c
	${SPARSE_DIR}cs_gaxpy.c
	${SPARSE_DIR}cs_happly.c
	${SPARSE_DIR}cs_house.c
	${SPARSE_DIR}cs_ipvec.c
	${SPARSE_DIR}cs_leaf.c
	${SPARSE_DIR}cs_load.c
	${SPARSE_DIR}cs_lsolve.c
	${SPARSE_DIR}cs_ltsolve.c
	${SPARSE_DIR}cs_lu.c
	${SPARSE_DIR}cs_lusol.c
	${SPARSE_DIR}cs_malloc.c
	${SPARSE_DIR}cs_maxtrans.c
	${SPARSE_DIR}cs_multiply.c
	${SPARSE_DIR}cs_norm.c
	${SPARSE_DIR}cs_permute.c
	${SPARSE_DIR}cs_pinv.c
	${SPARSE_DIR}cs_post.c
	${SPARSE_DIR}cs_print.c
	${SPARSE_DIR}cs_pvec.c
	${SPARSE_DIR}cs_qr.c
	${SPARSE_DIR}cs_qrsol.c
	${SPARSE_DIR}cs_randperm.c
	${SPARSE_DIR}cs_reach.c
	${SPARSE_DIR}cs_scatter.c
	${SPARSE_DIR}cs_scc.c
	${SPARSE_DIR}cs_schol.c
	${SPARSE_DIR}cs_spsolve.c
	${SPARSE_DIR}cs_sqr.c
	${SPARSE_DIR}cs_symperm.c
	${SPARSE_DIR}cs_tdfs.c
	${SPARSE_DIR}cs_transpose.c
	${SPARSE_DIR}cs_updown.c
	${SPARSE_DIR}cs_usolve.c
	${SPARSE_DIR}cs_util.c
	${SPARSE_DIR}cs_utsolve.c
	)
# force suitesparse to be compiled in CXX mode. This is necessary because MSVC
# will compile them in C mode due to their .c file extension but that in turn
# will fail because of the line "#define cs_complex_t double _Complex" in cs.h
# does not work in C mode in MSVC
# this is because MSVC does not support C99...
SET_SOURCE_FILES_PROPERTIES( ${SPARSE_SOURCES} PROPERTIES LANGUAGE CXX )

add_library(cxsparse_objects OBJECT ${SPARSE_SOURCES})

# compile with -fPIC so that the result can be made into a shared library
set_property(TARGET cxsparse_objects PROPERTY POSITION_INDEPENDENT_CODE 1)

add_library(cxsparse_static STATIC $<TARGET_OBJECTS:cxsparse_objects>)
add_library(cxsparse_shared SHARED $<TARGET_OBJECTS:cxsparse_objects>)

set_target_properties(cxsparse_static PROPERTIES OUTPUT_NAME cxsparse CLEAN_DIRECT_OUTPUT 1)
set_target_properties(cxsparse_shared PROPERTIES OUTPUT_NAME cxsparse CLEAN_DIRECT_OUTPUT 1)
