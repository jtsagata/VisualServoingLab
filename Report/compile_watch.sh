#!/bin/bash

rubber --clean Report.tex
rubber -d Report.tex
while true; do inotifywait -e modify *.tex; rubber -d Report.tex; done
