QUARTUS_PROJECT_NAME = Benchmark_project

QUARTUS_PROJECT_FILE = $(QUARTUS_PROJECT_NAME).qsf

SOURCE_FILES_FROM_QSF=$(shell sed -n 's/set_global_assignment -name [^ \t]*_FILE \([^ ]*\)\(\| -.*\)$$/\1/p' < $(QUARTUS_PROJECT_FILE) )

#$(warning $(SOURCE_FILES_FROM_QSF))

all : $(QUARTUS_PROJECT_NAME).svf

# do not bail out if files do not exist
SDC_FILE Benchmark_project.sdc:
SDC_FILE Benchmark_project.out.sdc:

output_files/$(QUARTUS_PROJECT_NAME).sof : update_core_sources
output_files/$(QUARTUS_PROJECT_NAME).sof : $(QUARTUS_PROJECT_FILE) $(SOURCE_FILES_FROM_QSF)
	quartus_sh --flow compile $<
	@awk '/; File Name with User-Entered Path/,/^$$/' \
	    <output_files/$(QUARTUS_PROJECT_NAME).map.rpt | \
	    sed -n -e 's/^;[^;]*;[^;]*;[^;]*; \(\/[^; ]*\) *;.*$$/\1/p'  \
	    >$(QUARTUS_PROJECT_NAME).dep.tmp || true
	@sed -n -e 's#^\(.*\)$$#\1:#p' <$(QUARTUS_PROJECT_NAME).dep.tmp \
		>$(QUARTUS_PROJECT_NAME).dep || true
	@sed -n -e 's#^\(.*\)$$#$@:\1#p' \
		<$(QUARTUS_PROJECT_NAME).dep.tmp >>$(QUARTUS_PROJECT_NAME).dep || true
	@rm $(QUARTUS_PROJECT_NAME).dep.tmp


$(QUARTUS_PROJECT_NAME).svf : output_files/$(QUARTUS_PROJECT_NAME).sof
	quartus_cpf -c -q 12MHz -g 3.3 -n v $< $@

update_core_sources:
	@find ../../src -name '*.vhd' | \
	sed -e 's/^\(.\+\)$$/set_global_assignment -name VHDL_FILE \1/' \
	> ctu_can_fd_core_sources.sf
	@awk ' \
	/\#\#\# CTU CAN FD core sources list - begin \#\#\#/  \
	{print; system("cat ctu_can_fd_core_sources.sf"); banner=1; next} '' \
	/\#\#\# CTU CAN FD core sources list - end \#\#\#/ {banner=0}  \
	banner {next} '' \
	{print} \
	' $(QUARTUS_PROJECT_FILE) >$(QUARTUS_PROJECT_FILE).tmp
	@if ! cmp $(QUARTUS_PROJECT_FILE) $(QUARTUS_PROJECT_FILE).tmp ; then \
		mv -f $(QUARTUS_PROJECT_FILE).tmp $(QUARTUS_PROJECT_FILE) ; \
	fi
	@rm -f $(QUARTUS_PROJECT_FILE).tmp

clean:
	rm -f *.rpt *.sopcinfo *.bak *.dep *.orig *.svf *.qws ctu_can_fd_core_sources.sf
	rm -rf output_files incremental_db .qsys_edit pcie_core greybox_tmp db

.PHONNY: clean all update_core_sources

-include $(QUARTUS_PROJECT_NAME).dep
