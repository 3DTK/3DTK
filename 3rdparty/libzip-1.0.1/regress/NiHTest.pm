package NiHTest;

use strict;
use warnings;

use Cwd;
use File::Copy;
use File::Path qw(mkpath);
use IPC::Open3;
use Symbol 'gensym';
use UNIVERSAL;

use Data::Dumper qw(Dumper);

#  NiHTest -- package to run regression tests
#  Copyright (C) 2002-2014 Dieter Baron and Thomas Klausner
#
#  This file is part of ckmame, a program to check rom sets for MAME.
#  The authors can be contacted at <ckmame@nih.at>
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#  3. The names of the authors may not be used to endorse or promote
#     products derived from this software without specific prior
#     written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS
#  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
#  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
#  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
#  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# runtest TESTNAME
#
# files:
#   TESTNAME.test: test scenario
#
# test scenario:
#    Lines beginning with # are comments.
#
#    The following commands are recognized; return and args must
#    appear exactly once, the others are optional.
#
#	args ARGS
#	    run program with command line arguments ARGS
#
#	description TEXT
#	    description of what test is for
#
#	features FEATURE ...
#	    only run test if all FEATUREs are present, otherwise skip it.
#
#	file TEST IN OUT
#	    copy file IN as TEST, compare against OUT after program run.
#
#	file-del TEST IN
#	    copy file IN as TEST, check that it is removed by program.
#
#	file-new TEST OUT
#	    check that file TEST is created by program and compare
#	    against OUT.
#
#	mkdir MODE NAME
#	    create directory NAME with permissions MODE.
#
#	pipein COMMAND ARGS ...
#	    pipe output of running COMMAND to program's stdin.
#
#	preload LIBRARY
#	    pre-load LIBRARY before running program.
#
#	program PRG
#	    run PRG instead of ckmame.
#
#	return RET
#	    RET is the expected exit code
#
#	setenv VAR VALUE
#	    set environment variable VAR to VALUE.
#
#	stderr TEXT
#	    program is expected to produce the error message TEXT.  If
#	    multiple stderr commands are used, the messages are
#	    expected in the order given.
#
#       stderr-replace REGEX REPLACEMENT
#           run regex replacement over expected and got stderr output.
#
#	stdout TEXT
#	    program is expected to print TEXT to stdout.  If multiple
#	    stdout commands are used, the messages are expected in
#	    the order given.
#
#	touch MTIME FILE
#	    set last modified timestamp of FILE to MTIME (seconds since epoch).
#	    If FILE doesn't exist, an empty file is created.
#
#	ulimit C VALUE
#	    set ulimit -C to VALUE while running the program.
#
# exit status
#	runtest uses the following exit codes:
#	    0: test passed
#	    1: test failed
#	    2: other error
#	   77: test was skipped
#
# environment variables:
#   RUN_GDB: if set, run gdb on program in test environment
#   KEEP_BROKEN: if set, don't delete test environment if test failed
#   NO_CLEANUP: if set, don't delete test environment
#   SETUP_ONLY: if set, exit after creating test environment
#   VERBOSE: if set, be more verbose (e. g., output diffs)

my %EXIT_CODES = (
	PASS => 0,
	FAIL => 1,
	SKIP => 77,
	ERROR => 99
    );

sub new {
	my $class = UNIVERSAL::isa ($_[0], __PACKAGE__) ? shift : __PACKAGE__;
	my $self = bless {}, $class;
	
	my ($opts) = @_;

	$self->{default_program} = $opts->{default_program};
	$self->{zipcmp} = $opts->{zipcmp} // 'zipcmp';
	$self->{zipcmp_flags} = $opts->{zipcmp_flags};

	$self->{directives} = {
		args => { type => 'string...', once => 1, required => 1 },
		description => { type => 'string', once => 1 },
		features => { type => 'string...', once => 1 },
		file => { type => 'string string string' },
		'file-del' => { type => 'string string' },
		'file-new' => { type => 'string string' },
		mkdir => { type => 'string string' },
		pipein => { type => 'string', once => 1 },
		preload => { type => 'string', once => 1 },
		program => { type => 'string', once => 1 },
		'return' => { type => 'int', once => 1, required => 1 },
		setenv => { type => 'string string' },
		stderr => { type => 'string' },
		'stderr-replace' => { type => 'string string' },
		stdout => { type => 'string' },
		touch => { type => 'int string' },
		ulimit => { type => 'char string' }
	};
	
	$self->{compare_by_type} = {};
	$self->{copy_by_type} = {};
	$self->{hooks} = {};

	$self->add_comparator('zip/zip', \&comparator_zip);
	
	$self->{srcdir} = $opts->{srcdir} // $ENV{srcdir};
	
	if (!defined($self->{srcdir}) || $self->{srcdir} eq '') {
		$self->{srcdir} = `sed -n 's/^srcdir = \(.*\)/\1/p' Makefile`;
		chomp($self->{srcdir});
	}
	
	$self->{in_sandbox} = 0;
	
	$self->{verbose} = $ENV{VERBOSE};
	$self->{keep_broken} = $ENV{KEEP_BROKEN};
	$self->{no_cleanup} = $ENV{NO_CLEANUP};
	$self->{setup_only} = $ENV{SETUP_ONLY};

	return $self;
}


sub add_comparator {
	my ($self, $ext, $sub) = @_;
	
	return $self->add_file_proc('compare_by_type', $ext, $sub);
}


sub add_copier {
	my ($self, $ext, $sub) = @_;

	return $self->add_file_proc('copy_by_type', $ext, $sub);
}


sub add_directive {
	my ($self, $name, $def) = @_;
	
	if (exists($self->{directives}->{$name})) {
		$self->die("directive $name already defined");
	}
	
	# TODO: validate $def
	
	$self->{directives}->{$name} = $def;
	
	return 1;
}


sub add_file_proc {
	my ($self, $proc, $ext, $sub) = @_;

	$self->{$proc}->{$ext} = [] unless (defined($self->{$proc}->{$ext}));
	unshift @{$self->{$proc}->{$ext}}, $sub;

	return 1;
}


sub add_hook {
	my ($self, $hook, $sub) = @_;
	
	$self->{hooks}->{$hook} = [] unless (defined($self->{hooks}->{$hook}));
	push @{$self->{hooks}->{$hook}}, $sub;

	return 1;
}


sub end {
	my ($self, @results) = @_;

	my $result = 'PASS';

	for my $r (@results) {
		if ($r eq 'ERROR' || ($r eq 'FAIL' && $result ne 'ERROR')) {
			$result = $r;
		}
	}

	$self->end_test($result);
}


sub run {
	my ($self, @argv) = @_;

	$self->setup(@argv);

	$self->end($self->runtest());
}


sub runtest {
	my ($self, $tag) = @_;

	$ENV{TZ} = "UTC";
	$ENV{LC_CTYPE} = "C";
	$self->sandbox_create($tag);
	$self->sandbox_enter();
	
	my $ok = 1;
	$ok &= $self->copy_files();
	$ok &= $self->run_hook('post_copy_files');
	$ok &= $self->touch_files();
	$ok &= $self->run_hook('prepare_sandbox');
	return 'ERROR' unless ($ok);

	if ($self->{setup_only}) {
		$self->sandbox_leave();
		return 'SKIP';
	}

	for my $env (@{$self->{test}->{'setenv'}}) {
		$ENV{$env->[0]} = $env->[1];
	}
	if (defined($self->{test}->{'preload'})) {
		$ENV{LD_PRELOAD} = cwd() . "/../.libs/$self->{test}->{'preload'}";
	}

	$self->run_program();

	for my $env (@{$self->{test}->{'setenv'}}) {
		delete ${ENV{$env->[0]}};
	}
	if (defined($self->{test}->{'preload'})) {
		delete ${ENV{LD_PRELOAD}};
	}

	if ($self->{test}->{stdout}) {
		$self->{expected_stdout} = [ @{$self->{test}->{stdout}} ];
	}
	else {
		$self->{expected_stdout} = [];
	}
	if ($self->{test}->{stderr}) {
		$self->{expected_stderr} = [ @{$self->{test}->{stderr}} ];
	}
	else {
		$self->{expected_stderr} = [];
	}

	$self->run_hook('post_run_program');

	my @failed = ();
	
	if ($self->{exit_status} != $self->{test}->{return} // 0) {
		push @failed, 'exit status';
		if ($self->{verbose}) {
			print "Unexpected exit status:\n";
			print "-" . ($self->{test}->{return} // 0) . "\n+$self->{exit_status}\n";
		}
	}
	
	if (!$self->compare_arrays($self->{expected_stdout}, $self->{stdout}, 'output')) {
		push @failed, 'output';
	}
	if (!$self->compare_arrays($self->{expected_stderr}, $self->{stderr}, 'error output')) {
		push @failed, 'error output';
	}
	if (!$self->compare_files()) {
		push @failed, 'files';
	}
	
	$self->{failed} = \@failed;
	
	$self->run_hook('checks');
	
	my $result = scalar(@{$self->{failed}}) == 0 ? 'PASS' : 'FAIL';

	$self->sandbox_leave();
	if (!($self->{no_cleanup} || ($self->{keep_broken} && $result eq 'FAIL'))) {
		$self->sandbox_remove();
	}

	$self->print_test_result($tag, $result, join ', ', @{$self->{failed}});

	return $result;
}


sub setup {
	my ($self, @argv) = @_;
	
	if (scalar(@argv) != 1) {
		print STDERR "Usage: $0 testcase\n";
		exit(1);
	}
	
	my $testcase = shift @argv;

	$testcase .= '.test' unless ($testcase =~ m/\.test$/);

	my $testcase_file = $self->find_file($testcase);
	
	$self->die("cannot find test case $testcase") unless ($testcase_file);
	
	$testcase =~ s,^(?:.*/)?([^/]*)\.test$,$1,;
	$self->{testname} = $testcase;

	$self->die("error in test case definition") unless $self->parse_case($testcase_file);
	
	$self->check_features_requirement() if ($self->{test}->{features});
}


#
# internal methods
#

sub add_file {
	my ($self, $file) = @_;
	
	if (defined($self->{files}->{$file->{destination}})) {
		$self->warn("duplicate specification for input file $file->{destination}");
		return undef;
	}
        
	$self->{files}->{$file->{destination}} = $file;
	
	return 1;
}


sub check_features_requirement() {
	my ($self) = @_;
	
	### TODO: implement
	
	return 1;
}


sub comparator_zip {
	my ($self, $got, $expected) = @_;

	my @args = ($self->{zipcmp}, $self->{verbose} ? '-pv' : '-pq');
	push @args, $self->{zipcmp_flags} if ($self->{zipcmp_flags});
	push @args, ($expected, $got);
        
	my $ret = system(@args);
	
	return $ret == 0;
}


sub compare_arrays() {
	my ($self, $a, $b, $tag) = @_;
	
	my $ok = 1;
	
	if (scalar(@$a) != scalar(@$b)) {
		$ok = 0;
	}
	else {
		for (my $i = 0; $i < scalar(@$a); $i++) {
			if ($a->[$i] ne $b->[$i]) {
				$ok = 0;
				last;
			}
		}
	}
	
	if (!$ok && $self->{verbose}) {
		print "Unexpected $tag:\n";
		print "--- expected\n+++ got\n";

		diff_arrays($a, $b);
	}
	
	return $ok;
}


sub compare_file() {
	my ($self, $got, $expected) = @_;
	
	my $real_expected = $self->find_file($expected);
	unless ($real_expected) {
		$self->warn("cannot find expected result file $expected");
		return 0;
	}

	my $ok = $self->run_comparator($got, $real_expected);

	if (!defined($ok)) {
		my $ret;
		if ($self->{verbose}) {
			$ret = system('diff', '-u', $real_expected, $got);
		}
		else {
			$ret = system('cmp', '-s', $real_expected, $got);
		}
		$ok = ($ret == 0);
	}

	return $ok;
}


sub compare_files() {
	my ($self) = @_;
	
	my $ok = 1;
	
	my $ls;
	open $ls, "find . -type f -print |";
	unless ($ls) {
		# TODO: handle error
	}
	my @files_got = ();
	
	while (my $line = <$ls>) {
		chomp $line;
		$line =~ s,^\./,,;
		push @files_got, $line;
	}
	close($ls);
	
	@files_got = sort @files_got;
	my @files_should = ();
	
        for my $file (sort keys %{$self->{files}}) {
		push @files_should, $file if ($self->{files}->{$file}->{result} || $self->{files}->{$file}->{ignore});
	}

	$self->{files_got} = \@files_got;
	$self->{files_should} = \@files_should;

	unless ($self->run_hook('post_list_files')) {
		return 0;
	}
	
	$ok = $self->compare_arrays($self->{files_should}, $self->{files_got}, 'files');
	
	for my $file (@{$self->{files_got}}) {
		my $file_def = $self->{files}->{$file};
		next unless ($file_def && $file_def->{result});
		
		$ok &= $self->compare_file($file, $file_def->{result});
	}
	
	return $ok;
}


sub copy_files {
	my ($self) = @_;
	
	my $ok = 1;
	
	for my $filename (sort keys %{$self->{files}}) {
		my $file = $self->{files}->{$filename};
		next unless ($file->{source});

		my $src = $self->find_file($file->{source});
		unless ($src) {
			$self->warn("cannot find input file $file->{source}");
			$ok = 0;
			next;
		}

		if ($file->{destination} =~ m,/,) {
			my $dir = $file->{destination};
			$dir =~ s,/[^/]*$,,;
			if (! -d $dir) {
				mkpath($dir);
			}
		}

		my $this_ok = $self->run_copier($src, $file->{destination});
		if (defined($this_ok)) {
			$ok &= $this_ok;
		}
		else {
			unless (copy($src, $file->{destination})) {
				$self->warn("cannot copy $src to $file->{destination}: $!");
				$ok = 0;
			}
		}
	}

	if (defined($self->{test}->{mkdir})) {
		for my $dir_spec (@{$self->{test}->{mkdir}}) {
			my ($mode, $dir) = @$dir_spec;
			if (! -d $dir) {
				unless (mkdir($dir, oct($mode))) {
					$self->warn("cannot create directory $dir: $!");
					$ok = 0;
				}
			}
		}
	}
	
	$self->die("failed to copy input files") unless ($ok);
}


sub die() {
	my ($self, $msg) = @_;
	
	print STDERR "$0: $msg\n" if ($msg);
	
	$self->end_test('ERROR');
}


sub end_test {
	my ($self, $status) = @_;
	
	my $exit_code = $EXIT_CODES{$status} // $EXIT_CODES{ERROR};
	
	$self->exit($exit_code);
}



sub exit() {
	my ($self, $status) = @_;
	### TODO: cleanup
	
	exit($status);
}


sub find_file() {
	my ($self, $fname) = @_;
	
	for my $dir (('', "$self->{srcdir}/")) {
		my $f = "$dir$fname";
		$f = "../$f" if ($self->{in_sandbox} && $dir !~ m,^/,);
		
		return $f if (-f $f);
	}
	
	return undef;
}


sub get_extension {
	my ($self, $fname) = @_;

	my $ext = $fname;
	if ($ext =~ m/\./) {
		$ext =~ s/.*\.//;
	}
	else {
		$ext = '';
	}

	return $ext;
}


sub parse_args {
	my ($self, $type, $str) = @_;

	if ($type eq 'string...') {
		my $args = [];

		while ($str ne '') {
			if ($str =~ m/^\"/) {
				unless ($str =~ m/^\"([^\"]*)\"\s*(.*)/) {
					$self->warn_file_line("unclosed quote in [$str]");
					return undef;
				}
				push @$args, $1;
				$str = $2;
			}
			else {
				$str =~ m/^(\S+)\s*(.*)/;
				push @$args, $1;
				$str = $2;
			}
		}

		return $args;
	}
	elsif ($type =~ m/(\s|\.\.\.$)/) {
		my $ellipsis = 0;
		if ($type =~ m/(.*)\.\.\.$/) {
			$ellipsis = 1;
			$type = $1;
		}
		my @types = split /\s+/, $type;
		my @strs = split /\s+/, $str;
		
		if (!$ellipsis && scalar(@types) != scalar(@strs)) {
			$self->warn_file_line("expected " . (scalar(@types)) . " arguments, got " . (scalar(@strs)));
			return undef;
		}
		
		my $args = [];
		
		my $n = scalar(@types);
		for (my $i=0; $i<scalar(@strs); $i++) {
			my $val = $self->parse_args(($i >= $n ? $types[$n-1] : $types[$i]), $strs[$i]);
			return undef unless (defined($val));
			push @$args, $val;
		}
		
		return $args;
	}
	else {
		if ($type eq 'string') {
			return $str;
		}
		elsif ($type eq 'int') {
			if ($str !~ m/^\d+$/) {
				$self->warn_file_line("illegal int [$str]");
				return undef;
			}
			return $str+0;
		}
		elsif ($type eq 'char') {
			if ($str !~ m/^.$/) {
				$self->warn_file_line("illegal char [$str]");
				return undef;
			}
			return $str;
		}
		else {
			$self->warn_file_line("unknown type $type");
			return undef;
		}
	}
}


sub parse_case() {
	my ($self, $fname) = @_;
	
	my $ok = 1;
	
	open TST, "< $fname" or $self->die("cannot open test case $fname: $!");
	
	$self->{testcase_fname} = $fname;
	
	my %test = ();
	
	while (my $line = <TST>) {
		chomp $line;
		
		next if ($line =~ m/^\#/);
		
		unless ($line =~ m/(\S*)(?:\s(.*))?/) {
			$self->warn_file_line("cannot parse line $line");
			$ok = 0;
			next;
		}
		my ($cmd, $argstring) = ($1, $2//"");
		
		my $def = $self->{directives}->{$cmd};
		
		unless ($def) {
			$self->warn_file_line("unknown directive $cmd in test file");
			$ok = 0;
			next;
		}
		
		my $args = $self->parse_args($def->{type}, $argstring);

		if (!defined($args)) {
			$ok = 0;
			next;
		}
		
		if ($def->{once}) {
			if (defined($test{$cmd})) {
				$self->warn_file_line("directive $cmd appeared twice in test file");
			}
			$test{$cmd} = $args;
		}
		else {
			$test{$cmd} = [] unless (defined($test{$cmd}));
			push @{$test{$cmd}}, $args;
		}
	}

	close TST;
	
	return undef unless ($ok);
	
	for my $cmd (sort keys %test) {
		if ($self->{directives}->{$cmd}->{required} && !defined($test{$cmd})) {
			$self->warn_file("required directive $cmd missing in test file");
			$ok = 0;
		}
	}
	
	return undef unless ($ok);

	if (defined($test{'stderr-replace'}) && defined($test{stderr})) {
		$test{stderr} = [ map { $self->stderr_rewrite($test{'stderr-replace'}, $_); } @{$test{stderr}} ];
	}

	if (!defined($test{program})) {
		$test{program} = $self->{default_program};
	}

	$self->{test} = \%test;

	$self->run_hook('mangle_program');
	
	if (!$self->parse_postprocess_files()) {
		return 0;
	}

	return $self->run_hook('post_parse');
}


sub parse_postprocess_files {
	my ($self) = @_;
	
	$self->{files} = {};
	
	my $ok = 1;
	
	for my $file (@{$self->{test}->{file}}) {
		$ok = 0 unless ($self->add_file({ source => $file->[1], destination => $file->[0], result => $file->[2] }));
	}
	
	for my $file (@{$self->{test}->{'file-del'}}) {
		$ok = 0 unless ($self->add_file({ source => $file->[1], destination => $file->[0], result => undef }));
	}
	
	for my $file (@{$self->{test}->{'file-new'}}) {
		$ok = 0 unless ($self->add_file({ source => undef, destination => $file->[0], result => $file->[1] }));
	}
	
	return $ok;
}


sub print_test_result {
	my ($self, $tag, $result, $reason) = @_;

	if ($self->{verbose}) {
		print "$self->{testname}";
		print " ($tag)" if ($tag);
		print " -- $result";
		print ": $reason" if ($reason);
		print "\n";
	}
}


sub run_comparator {
	my ($self, $got, $expected) = @_;

	return $self->run_file_proc('compare_by_type', $got, $expected);
}


sub run_copier {
	my ($self, $src, $dest) = @_;

	return $self->run_file_proc('copy_by_type', $src, $dest);
}


sub run_file_proc {
	my ($self, $proc, $got, $expected) = @_;

	my $ext = ($self->get_extension($got)) . '/' . ($self->get_extension($expected));

	if (defined($self->{$proc}->{$ext})) {
		for my $sub (@{$self->{$proc}->{$ext}}) {
			my $ret = $sub->($self, $got, $expected);
			return $ret if (defined($ret));
		}
	}

	return undef;
}


sub run_hook {
	my ($self, $hook) = @_;
	
	my $ok = 1;
	
	if (defined($self->{hooks}->{$hook})) {
		for my $sub (@{$self->{hooks}->{$hook}}) {
			unless ($sub->($self, $hook)) {
				$self->warn("hook $hook failed");
				$ok = 0;
			}
		}
	}
	
	return $ok;
}


sub backslash_decode {
	my ($str) = @_;

	if ($str =~ m/\\/) {
		$str =~ s/\\a/\a/gi;
		$str =~ s/\\b/\b/gi;
		$str =~ s/\\f/\f/gi;
		$str =~ s/\\n/\n/gi;
		$str =~ s/\\r/\r/gi;
		$str =~ s/\\t/\t/gi;
		$str =~ s/\\v/\cK/gi;
		$str =~ s/\\s/ /gi;
		# TODO: \xhh, \ooo
		$str =~ s/\\(.)/$1/g;
	}

	return $str;
}


sub run_program {
	my ($self) = @_;
	
	my ($stdin, $stdout, $stderr);
	$stderr = gensym;

	my @cmd = ('../' . $self->{test}->{program}, map ({ backslash_decode($_); } @{$self->{test}->{args}}));

	### TODO: catch errors?
	
	my $pid = open3($stdin, $stdout, $stderr, @cmd);
	
	$self->{stdout} = [];
	$self->{stderr} = [];
        
        if ($self->{test}->{pipein}) {
                my $fh;
                open($fh, "$self->{test}->{pipein} |");
                if (!defined($fh)) {
                        $self->die("cannot run pipein command [$self->{test}->{pipein}: $!");
                }
                while (my $line = <$fh>) {
                        print $stdin $line;
                }
                close($fh);
                close($stdin);
        }
	
	while (my $line = <$stdout>) {
		chomp $line;
		push @{$self->{stdout}}, $line;
	}
	my $prg = $self->{test}->{program};
	$prg =~ s,.*/,,;
	while (my $line = <$stderr>) {
		chomp $line;

		$line =~ s/^[^: ]*$prg: //;
		if (defined($self->{test}->{'stderr-replace'})) {
			$line = $self->stderr_rewrite($self->{test}->{'stderr-replace'}, $line);
		}
		push @{$self->{stderr}}, $line;
	}
	
	waitpid($pid, 0);
	
	$self->{exit_status} = $? >> 8;
}


sub sandbox_create {
	my ($self, $tag) = @_;
	
	$tag = ($tag ? "-$tag" : "");
	$self->{sandbox_dir} = "sandbox-$self->{testname}$tag.d$$";
	
	$self->die("sandbox $self->{sandbox_dir} already exists") if (-e $self->{sandbox_dir});
	
	mkdir($self->{sandbox_dir}) or $self->die("cannot create sandbox $self->{sandbox_dir}: $!");
	
	return 1;
}


sub sandbox_enter {
	my ($self) = @_;
	
	$self->die("internal error: cannot enter sandbox before creating it") unless (defined($self->{sandbox_dir}));

	return if ($self->{in_sandbox});

	chdir($self->{sandbox_dir}) or $self->die("cant cd into sandbox $self->{sandbox_dir}: $!");
	
	$self->{in_sandbox} = 1;
}


sub sandbox_leave {
	my ($self) = @_;
	
	return if (!$self->{in_sandbox});
	
	chdir('..') or $self->die("cannot leave sandbox: $!");
	
	$self->{in_sandbox} = 0;
}


sub sandbox_remove {
	my ($self) = @_;

	my $ok = 1;
	unless (system('chmod', '-R', 'u+rwx', $self->{sandbox_dir}) == 0) {
		$self->warn("can't ensure that sandbox is writable: $!");
	}
	unless (system('rm', '-rf', $self->{sandbox_dir}) == 0) {
		$self->warn("can't remove sandbox: $!");
		$ok = 0;
	}
	
	return $ok;
}


sub touch_files {
	my ($self) = @_;
	
	my $ok = 1;
	
	if (defined($self->{test}->{touch})) {
		for my $args (@{$self->{test}->{touch}}) {
			my ($mtime, $fname) = @$args;
			
			if (!-f $fname) {
				my $fh;
				unless (open($fh, "> $fname") and close($fh)) {
					# TODO: error message
					$ok = 0;
					next;
				}
			}
			unless (utime($mtime, $mtime, $fname) == 1) {
				# TODO: error message
				$ok = 0;
			}
		}
	}
	
	return $ok;
}


sub warn {
	my ($self, $msg) = @_;
	
	print STDERR "$0: $msg\n";
}


sub warn_file {
	my ($self, $msg) = @_;
	
	$self->warn("$self->{testcase_fname}: $msg");
}


sub warn_file_line {
	my ($self, $msg) = @_;
	
	$self->warn("$self->{testcase_fname}:$.: $msg");
}

sub stderr_rewrite {
	my ($self, $pattern, $line) = @_;
	for my $repl (@{$pattern}) {
		$line =~ s/$repl->[0]/$repl->[1]/;
	}
	return $line;
}


# MARK: array diff

sub diff_arrays {
	my ($a, $b) = @_;

	my ($i, $j);
	for ($i = $j = 0; $i < scalar(@$a) || $j < scalar(@$b);) {
		if ($i >= scalar(@$a)) {
			print "+$b->[$j]\n";
			$j++;
		}
		elsif ($j >= scalar(@$b)) {
			print "-$a->[$i]\n";
			$i++;
		}
		elsif ($a->[$i] eq $b->[$j]) {
			print " $a->[$i]\n";
			$i++;
			$j++;
		}
		else {
			my ($off_a, $off_b) = find_best_offsets($a, $i, $b, $j);
			my ($off_b_2, $off_a_2) = find_best_offsets($b, $j, $a, $i);

			if ($off_a + $off_b > $off_a_2 + $off_b_2) {
				$off_a = $off_a_2;
				$off_b = $off_b_2;
			}

			for (my $off = 0; $off < $off_a; $off++) {
				print "-$a->[$i]\n";
				$i++;
			}
			for (my $off = 0; $off < $off_b; $off++) {
				print "+$b->[$j]\n";
				$j++;
			}
		}
	}

}

sub find_best_offsets {
	my ($a, $i, $b, $j) = @_;

	my ($best_a, $best_b);

	for (my $off_a = 0; $off_a < (defined($best_a) ? $best_a + $best_b : scalar(@$a) - $i); $off_a++) {
		my $off_b = find_entry($a->[$i+$off_a], $b, $j, defined($best_a) ? $best_a + $best_b - $off_a : scalar(@$b) - $j);

		next unless (defined($off_b));

		if (!defined($best_a) || $best_a + $best_b > $off_a + $off_b) {
			$best_a = $off_a;
			$best_b = $off_b;
		}
	}

	if (!defined($best_a)) {
		return (scalar(@$a) - $i, scalar(@$b) - $j);
	}
	
	return ($best_a, $best_b);
}

sub find_entry {
	my ($entry, $array, $start, $max_offset) = @_;

	for (my $offset = 0; $offset < $max_offset; $offset++) {
		return $offset if ($array->[$start + $offset] eq $entry);
	}

	return undef;
}

1;
