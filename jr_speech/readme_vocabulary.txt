You can also bring the file up in your favorite editor and add, delete or change some of
these words or phrases before proceeding to the next step. When you enter your
phrases, try not to mix upper and lower case and do not use punctuation marks. Also, if
want to include a number such as 54, spell it out like "fifty four".
Before we can use this corpus with PocketSphinx, we need to compile it into special
dictionary and pronunciation files. This can be done using the online CMU language
model (lm) tool located at:

http://www.speech.cs.cmu.edu/tools/lmtool-new.html

Follow the directions to upload your nav_commands.txt file, click the Compile
Knowledge Base button, then download the file labeled COMPRESSED TARBALL
that contains all the language model files. Extract these files into the config
subdirectory of the rbx1_speech package. The files will all begin with the same
number, such as 3026.dic , 3026.lm , etc. These files define your vocabulary as a
language model that PocketSphinx can understand. You can rename all these files to
something more memorable using a command like the following (the 4-digit number
will likely be different in your case):

$ roscd rbx1_speech/config
$ rename -f 's/3026/nav_commands/' *

Next, take a look at the voice_nav_commands.launch file found in the
rbx1_speech/launch subdirectory. It looks like the following