
all:


bump-upload:
	$(MAKE) bump
	$(MAKE) upload

bump: # v2
	bumpversion patch
	git push --tags
	git push

upload: # v3
	dts build_utils check-not-dirty
	dts build_utils check-tagged
	dt-check-need-upload --package aido-analyze-daffy make upload-do

upload-do:
	rm -f dist/*
	rm -rf src/*.egg-info
	python3 setup.py sdist
	twine upload --skip-existing --verbose dist/*
