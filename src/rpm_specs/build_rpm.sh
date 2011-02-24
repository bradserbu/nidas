#!/bin/sh

script=`basename $0`

dopkg=all
install=false

while [ $# -gt 0 ]; do
    case $1 in
        -i)
            install="true"
            ;;
        *)
            dopkg=$1
            ;;
    esac
    shift
done

source repo_scripts/repo_funcs.sh

topdir=`get_rpm_topdir`
rroot=`get_eol_repo_root`

log=/tmp/$script.$$
trap "{ rm -f $log; }" EXIT

set -o pipefail

get_version() 
{
    awk '/^Version:/{print $2}' $1
}

pkg=nidas
if [ $dopkg == all -o $dopkg == $pkg ];then
    version=`get_version ${pkg}.spec`
    tar czf $topdir/SOURCES/${pkg}-${version}.tar.gz --exclude .svn nidas
    rpmbuild -ba ${pkg}.spec | tee -a $log  || exit $?
fi

pkg=nidas-ael
if [ $dopkg == all -o $dopkg == $pkg ];then
    rpmbuild -ba ${pkg}.spec | tee -a $log  || exit $?
fi

# must specify nidas-bin in command line to build this package
pkg=nidas-bin
if [ $dopkg == $pkg ];then
    # Change topdir for a machine specific build.
    # So that we don't compile from scratch everytime, do not --clean the BUILD
    # tree with rpmbuild.  nidas-bin.spec %setup also has a -D option that
    # does not clear the BUILD tree before un-taring the source

    topdirx=${topdir}_`hostname`
    # echo "topdir=$topdirx"
    [ -d $topdirx/SOURCES ] || mkdir -p $topdirx/SOURCES
    [ -d $topdirx/BUILD ] || mkdir -p $topdirx/BUILD
    [ -d $topdirx/SRPMS ] || mkdir -p $topdirx/SRPMS
    [ -d $topdirx/RPMS ] || mkdir -p $topdirx/RPMS

    if [ `uname -m` == x86_64 ]; then
        export QT4DIR=/usr/lib64/qt4
    else
        export QT4DIR=/usr/lib/qt4
    fi

    PATH=$PATH:$QT4DIR/bin

    version=`get_version ${pkg}.spec`
    # tar option to rename  the top level directory: --transform="s/^nidas/nidas-bin/"
    tar czf $topdirx/SOURCES/${pkg}-${version}.tar.gz --exclude .svn -C ../../.. \
        nidas/src/SConstruct nidas/src/nidas nidas/src/site_scons \
        nidas/src/xml nidas/src/scripts
    rpmbuild -ba --define "_topdir $topdirx" ${pkg}.spec | tee -a $log  || exit $?
fi

echo "RPMS:"
egrep "^Wrote:" $log
rpms=`egrep '^Wrote:' $log | egrep /RPMS/ | awk '{print $2}'`
echo "rpms=$rpms"

if $install && [ -d $rroot ]; then
    echo "Moving rpms to $rroot"
    copy_rpms_to_eol_repo $rpms
elif $install; then
    echo "$rroot not found. Leaving RPMS in $topdir"
else
    echo "-i option not specified. RPMS will not be installed in $rroot"
fi

