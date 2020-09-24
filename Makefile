
all:


bump-upload:
	$(MAKE) bump
	$(MAKE) upload

bump: # v2
	bumpversion patch
	git push --tags
	git push

upload: # v3
	aido-check-not-dirty
	aido-check-tagged
	aido-check-need-upload --package aido-analyze-daffy make upload-do

upload-do:
	rm -f dist/*
	rm -rf src/*.egg-info
	python setup.py sdist
	twine upload --skip-existing --verbose dist/*

