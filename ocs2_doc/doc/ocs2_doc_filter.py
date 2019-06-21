##
# Doxygen filter for ocs2_doc
# This script modifies source files before doxygen parses them.
#
# Purpose: add grouping according to directory structure (e.g., ocs2_core)
#
##

import os
import sys
import re
import fnmatch
import inspect
import pathlib

class ocs2_doc_filter:

  ## Logging level: do not log anything.
  logNone   = 0
  ## Logging level: log errors only.
  logErrors = 1
  ## Logging level: log everything.
  logAll    = 2



  def __init__(self):
    ## Debug log file name.
    self.logFile = "ocs2_doc_filter.log"
    ## Error log file name.
    self.errorLogFile = "ocs2_doc_filter.error.log"
    ## Logging level.
    self.logLevel = self.logAll

  # Handles a file.
  def handleFile(self, fileName):
    if fnmatch.fnmatch(filename, '*.h') or fnmatch.fnmatch(filename, '*.hpp'):
      self.log('\nXXXXXXXXXX\nXX ' + filename + '\nXXXXXXXXXX\n\n')
      # Open the file. Use try to detect whether or not we have an actual file.
      try:
        with open(filename, 'r', encoding="utf-8") as inputFile:
          self.parseFile(inputFile)
        pass
      except IOError as e:
        self.logError('the file ' + filename + ' could not be opened for reading')
    else:
      # just pass on to doxygen
      try:
        with open(fileName, 'r', encoding="utf-8") as inputFile:
          self.log('\nXXXXXXXXXX\nXX ' + filename + ' --not edited--\nXXXXXXXXXX\n\n')
          theOutput = ''
          # print(inputFile.read())
          #[print(line) for line in inputFile]
          for line in inputFile:
            theOutput += line
          lines = theOutput.splitlines()
          for line in lines:
            try:
              print(line)
              # Our logger does not add extra line breaks so explicitly adding one to make the log more readable.
              self.log(line + '\n')
            except UnicodeEncodeError as e:
              self.logError("UnicodeEncodeError in file " + filename)
              self.logError(str(e))
      except IOError as e:
        self.logError('the file ' + filename + ' could not be opened for reading')

  # assumes c++ header file as inputFile
  def parseFile(self, inputFile):
    # Go through the input file line by line.

    numCurlyBracesOpen = 0
    startPattern = re.compile("\s*namespace\s+ocs2\s*{") # }
    foundNamespaceStart = False
    foundNamespaceEnd = False

    theOutput = ''

    for line in inputFile:
      if foundNamespaceEnd:
        # already done, just forward the rest of the file
        theOutput += line
      elif foundNamespaceStart:
        # found the start already but not the end yet
        # keep track of curly braces
        numCurlyBracesOpen += line.count('{')
        numCurlyBracesOpen -= line.count('}')
        if(numCurlyBracesOpen ==0):
          # {
          theOutput += "/*! @} End of Doxygen Groups*/\n"
          foundNamespaceEnd = True
        theOutput += line
      else:
        #did not find start yet: detect if line starts the ocs2 namespace
        theOutput += line
        if re.search(startPattern, line) is not None:
          foundNamespaceStart = True
          path = pathlib.Path(inputFile.name)
          groupName = "ocs2_unknown"
          for parentPath in reversed(path.parts):
            if re.search("ocs2_\w+", parentPath) is not None:
              groupName = parentPath
              break
          theOutput += "/*!\n *  @addtogroup "
          theOutput += groupName
          theOutput += " "
          theOutput += groupName
          theOutput += "\n *  @{\n */\n" # }
          numCurlyBracesOpen += line.count('{')
          numCurlyBracesOpen -= line.count('}')


    # Now that we've got all lines in the string let's split the lines and print out one by one.
    lines = theOutput.splitlines()
    for line in lines:
      print(line)
      # Our logger does not add extra line breaks so explicitly adding one to make the log more readable.
      self.log(line + '\n')

  def log(self, string):
    if self.logLevel >= self.logAll:
      with open(self.logFile, 'a') as theFile:
        theFile.write(string)

  def logError(self, string):
    if self.logLevel >= self.logErrors:
      with open(self.errorLogFile, 'a') as theFile:
        theFile.write(string)

converter = ocs2_doc_filter()
# Doxygen will gives us the file name
for filename in sys.argv[1:]:
  converter.handleFile(filename)

# end of file
