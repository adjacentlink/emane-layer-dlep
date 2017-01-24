XML= \
 emanelayerdlep.xml

EXTRA_DIST= \
 emanelayerdlep.xml.in

edit = sed \
 -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

emanelayerdlep.xml:	emanelayerdlep.xml.in
	$(edit) $< > $@

clean-local:
	-rm -f $(XML)

install-exec-hook:
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml
	-cp -f $(XML)  $(DESTDIR)$(datadir)/$(PACKAGE)/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/xml/*

