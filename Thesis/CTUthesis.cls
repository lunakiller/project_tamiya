%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CTUthesis -- LaTeX typesetting template for CTU thesis
%% Petr Cizek | petr.cizek@fel.cvut.cz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Class identification ---
\ProvidesClass{CTUthesis}[2018/04/03 version 1.00 CTU thesis template]
\NeedsTeXFormat{LaTeX2e}
% This class extends the article class
% Read all the documentclass options; pass them to article,
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions \relax
\LoadClass{article}

\RequirePackage{ifthen}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% load proper language package
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\ifthenelse{ \equal{\lang}{EN} }{
	\RequirePackage[english]{babel}
	\addto\captionsenglish{\renewcommand{\partname}{\sffamily Chapter}}
	\addto\captionsenglish{\renewcommand{\refname}{\sffamily References}}
	\newcommand{\appname}{\sffamily Appendix}
}{
	\RequirePackage[czech]{babel}
	\addto\captionsczech{\renewcommand{\partname}{\sffamily Kapitola}}
	\addto\captionsczech{\renewcommand{\refname}{\sffamily Literatura}}
	\newcommand{\appname}{\sffamily Příloha}
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of typesetting area
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\RequirePackage[left=30mm,right=24mm,top=30mm,bottom=30mm]{geometry}

\newlength{\titlewidth}
\setlength{\titlewidth}{\textwidth}
\addtolength{\titlewidth}{-0.7cm}
\newlength{\heightofhw}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of Colors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\RequirePackage[cmyk]{xcolor}
\definecolor{CTUblue}{cmyk}{1, 0.43, 0, 0} %CTU blue

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% formating of title page
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\newcommand{\TitlePage}{{
	\thispagestyle{empty}
	%vertical blue rule
	\noindent\parbox{0.7cm}{\color{CTUblue}\rule{0.35cm}{\textheight}}
	{\parbox[0][\textheight][t]{\titlewidth}{	
        %% CTU logo	 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \ifthenelse{ \equal{\lang}{EN} }{
    		\includegraphics[height=3.7cm]{logo/logo_cvut_en_inv}\\[1em]
        }{    
    		\includegraphics[height=3.7cm]{logo/logo_cvut_cz_inv}\\[1em]
        }
        %% faculty name %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		{\fontsize{14pt}{20pt}\selectfont
			\color{CTUblue} \bf \sffamily \faculty 
		\par}
	    \vspace{0.1em}
        %% department name %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		{\fontsize{14pt}{20pt}\selectfont
			\color{black} \bf \sffamily \department
		\par}
		\vspace{20mm}
        %% thesis type %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        {\fontsize{14pt}{20pt}\selectfont
		    \color{black} \bf \sffamily \worktype
		\par}
		\vspace{0.3em}
        %% thesis title %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		{\fontsize{22pt}{28pt}\selectfont
		    \color{CTUblue} \bf \sffamily 
		    \raggedright
		    \title
		\par}
		\vspace{0.8em}
        %% thesis subtitle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        {\fontsize{14pt}{18pt}\selectfont
		    \color{black} \sffamily 
            \raggedright		    
		    \subtitle
		\par}
		\vspace{0.3em}		
		%% author %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		{\fontsize{16pt}{18pt}\selectfont
    		\color{black} \bf \sffamily \author
		\par}
		\vfill{}
		%% date %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		{\fontsize{14pt}{18pt}\selectfont
		    \color{black} \bf \sffamily \date
	    \par}
	    \vspace{0.1em}
	    %% supervisor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		{\fontsize{14pt}{18pt}\selectfont
		    \color{black} {\bf \sffamily Supervisor:} \sffamily \supervisor
	    \par}
	    \vspace{0.1em}
	    %% supervisor specialist %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%		{\fontsize{14pt}{18pt}\selectfont
%		    \color{black} {\bf \sffamily Supervisor specialist:} \sffamily \supervisorspecialist
%	    \par}
	}}
	\newpage
}}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of header and footer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\RequirePackage{lastpage}
\RequirePackage{tikz}
\RequirePackage{fancyhdr}
    %% definition of pagestyle - normal text - blue header text %%%%%%%%%%%%%
    \fancypagestyle{pagestyle_text}{
        \fancyhead{}
        \fancyfoot{}
        %% footer settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \fancyfoot[CO,CE]{ %center odd, center even
            \thepage
        }
        %\fancyfoot[CO,CE]{ %center odd, center even
        %    \thepage$/$\pageref*{LastPage}
        %}
        \fancyfoot[RO,RE,LO,LE]{} %right odd, right even, left odd, left even
        
        %% header settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \renewcommand{\headrulewidth}{0pt}
        \fancyhead[RO]{\color{CTUblue} \slshape \rightmark }
	    \fancyhead[RE]{}
    	\fancyhead[LE]{\color{CTUblue} \slshape \rightmark }
	    \fancyhead[LO]{}
    }
    %% definition of pagestyle with part heading - no header text %%%%%%%%%%%
    \fancypagestyle{pagestyle_chapter}{
        \fancyhead{}
        \fancyfoot{}
        %% footer settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \fancyfoot[CO,CE]{
            \thepage
        }
        \fancyfoot[RO,RE,LO,LE]{} %right odd, right even, left odd, left even
        %\fancyfoot[CO,CE]{
        %    \thepage$/$\pageref*{LastPage}
        %}
        \fancyfoot[RO,RE,LO,LE]{} %right odd, right even, left odd, left even
        
        %% header settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \renewcommand{\headrulewidth}{0pt}
        \fancyhead[RO]{}
	    \fancyhead[RE]{}
    	\fancyhead[LE]{}
	    \fancyhead[LO]{}
        }
    %%definition of pagestyle with vertical blue line on the left side
    \fancypagestyle{pagestyle_blue_rule}{
        \fancyhead{}
        \fancyfoot{}
        %% footer settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \fancyfoot[C]{\thepage \qquad \qquad \qquad}
        %% header settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        \renewcommand{\headrulewidth}{0pt}
        \fancyhead[L]{
            \hspace{-0.9cm}
            \begin{tikzpicture}
            \color{CTUblue} \draw [line width=0.35cm] (0,-1) -- (0,-1.06\textheight);
            \end{tikzpicture}
            \vspace{-1.06\textheight}
        }
   }

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of part, section, subsection titles style - blue vertical rule
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% redefinition of part(sffamily vertical blue rule) %%%%%%%%%%
\makeatletter
\let\stdpart\part
\renewcommand*\part{%
  \@ifstar{\starpart}{\@dblarg\nostarpart}}
\newcommand*\starpart[1]{
	\thispagestyle{pagestyle_chapter}
	\settoheight{\heightofhw}{\parbox{\titlewidth}{\Huge{{\sffamily #1}}}}
	%%vertical blue rule
	\noindent\parbox{0.7cm}{\color{CTUblue}\rule{0.35cm}{2.3\heightofhw}}
	%%chapter name
	{\parbox[0][3cm][c]{\titlewidth}{
		\stdpart*{{\sffamily #1}}
	}}\\[1.25cm]							% <--
  	\sectionmark{#1}
  	\markboth{#1}{#1}
  	\addcontentsline{toc}{part}{\sffamily{\hspace{1.5em}#1}}
}
\def\nostarpart[#1]#2{
	\thispagestyle{pagestyle_chapter}
	\settoheight{\heightofhw}{\parbox{\titlewidth}{\Huge{{Chapter X\\ \sffamily #2}}}}
	%%vertical blue rule
	\noindent\parbox{0.7cm}{\color{CTUblue}\rule{0.35cm}{2.1\heightofhw}}
	%%chapter name
	\parbox{\titlewidth}{
  		\stdpart[{#1}]{{\sffamily #2}}
  	}\\[1.25cm]							% <--
  	\sectionmark{#2}
  	\markboth{\thepart. #2}{\thepart. #2}
}
\makeatother

%% redefinition for sffamily font in section and subsection title %%%%%%%%%%%%%%%%%%%%%%%%%%
\makeatletter
%% formating of the section title %%%%%%%%%%%%%%%%%%%%%%%%%%
\renewcommand\section{\@startsection {section}{1}{\z@}%
                                   {-3.5ex \@plus -1ex \@minus -.2ex}%vspace before section name
                                   {1.0ex \@plus.2ex}%vspace after section name
                                   {\normalfont\Large\bfseries\sffamily}} %formating of section title
%% formating of the subsection title %%%%%%%%%%%%%%%%%%%%%%%
\renewcommand\subsection{\@startsection {subsection}{2}{\z@}%
                                   {-3.5ex \@plus -1ex \@minus -.2ex}%vspace before subsection name
                                   {1.0ex \@plus.2ex}%vspace after subsection name
                                   {\normalfont\large\bfseries\sffamily}} %formating of subsection title
                                   
%% vertical blue rule in front of section and subsection name %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\newcommand{\vertrule}{{\color{CTUblue}\rule{0.35cm}{0.8em}}\hspace{0.5cm}}
%% redefinition of the inbuild section function to show blue box (vertrule) infront of the section name
\def\@sect#1#2#3#4#5#6[#7]#8{%
  \ifnum #2>\c@secnumdepth
    \let\@svsec\@empty
  \else
    \refstepcounter{#1}%
    \protected@edef\@svsec{\@seccntformat{#1}\relax}%
  \fi
  \@tempskipa #5\relax
  \ifdim \@tempskipa>\z@
    \begingroup
      #6{%
        \@hangfrom{\vertrule \hskip #3\relax\@svsec}%
          \interlinepenalty \@M #8\@@par}%
    \endgroup
    \csname #1mark\endcsname{#7}%
    \addcontentsline{toc}{#1}{%
      \ifnum #2>\c@secnumdepth \else
        \protect\numberline{\csname the#1\endcsname}%
      \fi
      #7}%
  \else
    \def\@svsechd{%
      #6{\hskip #3\relax
      \@svsec #8}%
      \csname #1mark\endcsname{#7}%
      \addcontentsline{toc}{#1}{%
        \ifnum #2>\c@secnumdepth \else
          \protect\numberline{\csname the#1\endcsname}%
        \fi
        #7}}%
  \fi
  \@xsect{#5}
  %% add section name to be displayed in the header %%%%%%%%%%%%%%%
  \markboth{\thesection\,#7}{\thesection\,#7}
  }
\makeatother

%% if you don't want to use blue vertical rule in front of section names, comment the above code and uncomment the code below -> sffamily font in section names
%redefinition of section(sffamily)
%\makeatletter
%\renewcommand \thesection {\sffamily\@arabic\c@section}
%\let\stdsection\section
%\renewcommand*\section{%
%  \@ifstar{\starsection}{\@dblarg\nostarsection}}
%\newcommand*\starsection[1]{\stdsection*{{\sffamily #1}}}
%\def\nostarsection[#1]#2{\stdsection[{#1}]{{\sffamily #2}}}
%\makeatother
%
%%redefinition of subsection(sffamily)
%\makeatletter
%\let\stdsubsection\subsection
%\renewcommand*\subsection{%
%  \@ifstar{\starsubsection}{\@dblarg\nostarsubsection}}
%\newcommand*\starsubsection[1]{\stdsubsection*{{\sffamily #1}}}
%\def\nostarsubsection[#1]#2{\stdsubsection[{#1}]{{\sffamily #2}}}
%\makeatother
%
%%redefinition of subsubsection(sffamily)
%\makeatletter
%\let\stdsubsubsection\subsubsection
%\renewcommand*\subsubsection{%
%  \@ifstar{\starsubsubsection}{\@dblarg\nostarsubsubsection}}
%\newcommand*\starsubsubsection[1]{\stdsubsubsection*{{\sffamily #1}}}
%\def\nostarsubsubsection[#1]#2{\stdsubsubsection[{#1}]{{\sffamily #2}}}
%\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of part, section, subsection numbering and appearence in table of contents
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%definition of numbering - numbers are going in order Part.Section.Subsection.Subsubsection...
\makeatletter
\@addtoreset{section}{part} %reseting section counter with every chapter

\renewcommand\thepart{\sffamily\arabic{part}}
\renewcommand\thesection{\thepart.\sffamily\arabic{section}}
\renewcommand\thesubsection{\thesection.\sffamily\arabic{subsection}}
\makeatother

%% formating of the table of content %%%%%%%%%%%%%%%%%%%%%%%%%%
\makeatletter
\let\stdlpart\l@part
\renewcommand*\l@part{\vspace{-1em}\stdlpart}		%tab of sections
\renewcommand*\l@section{\@dottedtocline{1}{2.2em}{2em}}		%tab of sections
\renewcommand*\l@subsection{\@dottedtocline{2}{4.2em}{2.7em}}	%tab of subsections 4.2 = 2.2 + 2 from the previous line
\renewcommand*\l@subsubsection{\@dottedtocline{3}{6.9em}{3.6em}}%tab of subsubsections 6.9 = 4.2 + 2.7 from the previous line
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of the main environments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\newenvironment{document_head}{%at beginning
    \pagestyle{pagestyle_blue_rule}
    \newgeometry{ left=38mm,right=24mm,top=30mm,bottom=30mm, twoside=false}
    \pagenumbering{roman}
    }{%at end
    \restoregeometry
    \clearpage
    }
    
\newenvironment{document_body}{%at beginning
    \pagestyle{pagestyle_text}
    \pagenumbering{arabic}
    }{%at end
    \clearpage
    }
        
\newenvironment{document_appendices}{%at beginning
    \pagestyle{pagestyle_text}
    \setcounter{part}{0}
    \renewcommand{\thepart}{\Alph{part}}  
    \renewcommand{\partname}{\appname} 
    }{%at end
    \clearpage
    }
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of paragraph skip and indent/offset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\setlength{\parskip}{1em}			% <--
%\setlength{\parindent}{10pt}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% redefinition of bibliography
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\let\stdbibliography\bibliography
\renewcommand{\bibliography}[1]{
    \part*{\refname}
    \renewcommand{\refname}{}
    \vspace{-2cm}
    \stdbibliography{#1}
}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% setting for hyperreferences
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% hypperreferences in text %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\RequirePackage[
	bookmarks=true,
	hypertexnames=false,
	breaklinks=true,
	colorlinks=false,
	urlcolor=blue,
	linktoc=section,
	linktocpage=true,
	linkbordercolor={1 0.8 0},
	citebordercolor={0.36 0.54 0.76}%cyan%{0 0.68 0.84}%{0 0.36 0.72}%{0 1 0}
]{hyperref}


%% PDF properties %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\hypersetup{  
	pdfauthor={\author},
	pdftitle={\title},
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% definition of appendix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\renewcommand{\appendix}[1]{\part{#1}}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% named citation style -- adaptet from ijcai.sty style
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file implements citations for the ``named'' bibliography style.
%  Place it in a file called named.sty in the TeX search path.  (Placing it
% in the same directory as the LaTeX document should also work.)

% Prepared by Peter F. Patel-Schneider, with the assistance of several,
% since forgotten, LaTeX hackers.
% This style is NOT guaranteed to work.  It is provided in the hope
% that it will make the preparation of papers easier.
%
% There are undoubtably bugs in this style.  If you make bug fixes,
% improvements, etc.  please let me know.  My e-mail address is:
%       pfps@research.att.com

% The preparation of this file was supported by Schlumberger Palo Alto
% Research and AT\&T Bell Laboratories.

% This file can be modified and used in other conferences as long
% as credit to the authors and supporting agencies is retained, this notice
% is not changed, and further modification or reuse is not restricted.

% The ``named'' bibliography style creates citations with labels like
%       \citeauthoryear{author-info}{year}
% these labels are processed by the following commands:
%       \cite{keylist}
%               which produces citations with both author and year,
%               enclosed in square brackets
%       \shortcite{keylist}
%               which produces citations with year only,
%               enclosed in square brackets
%       \citeauthor{key}
%               which produces the author information only
%       \citeyear{key}
%               which produces the year information only
\makeatletter
\def\leftcite{\@up[}\def\rightcite{\@up]}

\def\cite{\def\citeauthoryear##1##2{\def\@thisauthor{##1}%
             \ifx \@lastauthor \@thisauthor \relax \else##1, \fi ##2}\@icite}
\def\shortcite{\def\citeauthoryear##1##2{##2}\@icite}

\def\citeauthor{\def\citeauthoryear##1##2{##1}\@nbcite}
\def\citeyear{\def\citeauthoryear##1##2{##2}\@nbcite}

% internal macro for citations with [] and with breaks between citations
% used in \cite and \shortcite
\def\@icite{\leavevmode\def\@citeseppen{-1000}%
 \def\@cite##1##2{\leftcite\nobreak\hskip 0in{##1\if@tempswa , ##2\fi}\rightcite}%
 \@ifnextchar [{\@tempswatrue\@citex}{\@tempswafalse\@citex[]}}
% internal macro for citations without [] and with no breaks
% used in \citeauthor and \citeyear
\def\@nbcite{\leavevmode\def\@citeseppen{1000}%
 \def\@cite##1##2{{##1\if@tempswa , ##2\fi}}%
 \@ifnextchar [{\@tempswatrue\@citex}{\@tempswafalse\@citex[]}}

% don't box citations, separate with ; and a space
% also, make the penalty between citations a parameter, 
% it may be a good place to break
\def\@citex[#1]#2{%
  \def\@lastauthor{}\def\@citea{}%
  \@cite{\@for\@citeb:=#2\do
    {\@citea\def\@citea{;\penalty\@citeseppen\ }%
     \if@filesw\immediate\write\@auxout{\string\citation{\@citeb}}\fi
     \@ifundefined{b@\@citeb}{\def\@thisauthor{}{\bf ?}\@warning
       {Citation `\@citeb' on page \thepage \space undefined}}%
     {\csname b@\@citeb\endcsname}\let\@lastauthor\@thisauthor}}{#1}}

% raise the brackets in bibliography labels
\def\@biblabel#1{\def\citeauthoryear##1##2{##1, ##2}\@up{[}#1\@up{]}\hfill}

\def\@up#1{\leavevmode\raise.2ex\hbox{#1}}
\makeatother
