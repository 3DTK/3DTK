#bash completion for show and slam6d

#have show &&
_show()
{
    local cur prev opts filetype

    COMPREPLY=()
    cur=`_get_cword`
    prev=${COMP_WORDS[COMP_CWORD-1]}
    
    ## split the long options
    _split_longopt

    # options list
    opts="-e -s -f --format -F --fps -m --max -M --min -O --octree -r --reduce -R --reflectance --reflectivity -a --amplitude -d --deviation -h --height -T --type --saveOct --loadOct --advanced"
    filetype="uos uos_map uos_rgb uos_frames uos_map_frames old rts rts_map ifp riegl_txt riegl_rgb riegl_bin zahn ply wrl xyz zuf iais front x3d rxp ais"

    case $prev in
        -f|--format)
            COMPREPLY=( $( compgen -W "${filetype}" -- ${cur} ) )
            return 0
            ;;
    esac


    # reflectivity/reflectance are synonymous
    if [[ ${cur} == --ref* ]] ; then
        COMPREPLY=( "--reflectance" ) 
        return 0
    fi


    # relevant options to complete
    if [[ ${cur} == -* ]] ; then
      COMPREPLY=( $( compgen -W "${opts}"  -- "$cur" ) )
    fi
    
    
    # stolen from _cd function in bash_completion    
    _cd
} &&
complete -o dirnames -F _show show


_slam6D()
{
    local cur prev opts filetype

    COMPREPLY=()
    cur=`_get_cword`
    prev=${COMP_WORDS[COMP_CWORD-1]}
    
    ## split the long options
    _split_longopt

#  -n FILE, --net=FILE
#         specifies the file that includes the net structure for SLAM
#

    opts="-e -s -f --format -m --max -M --min -O --octree -r --reduce -R --random -q --quiet -Q --veryquiet -u --cuda -p --trustpose -n --net --metascan -L --loop6DAlgo -l --loopsize -I --iterSLAM -i --iter -G --epsICP --epsICP --exportAllPoints --DlastSLAM -D --distSLAM -d --dist --cache -C --clpairs -c --cldist -A --anim -a --algo "
    filetype="uos uos_map uos_rgb uos_frames uos_map_frames old rts rts_map ifp riegl_txt riegl_rgb riegl_bin zahn ply wrl xyz zuf iais front x3d rxp ais"

    case $prev in
        -f|--format)
            COMPREPLY=( $( compgen -W "${filetype}" -- ${cur} ) )
            return 0
            ;;
        -L|--loop6DAlgo)
            COMPREPLY=( $( compgen -W "0 1 2 3 4 5 6" -- ${cur} ) )
            return 0
            ;;
        -G|--graphSlam6DAlgo)
            COMPREPLY=( $( compgen -W "0 1 2 3 4 5 6" -- ${cur} ) )
            return 0
            ;;
        -a|--algo)
            COMPREPLY=( $( compgen -W "0 1 2 3 4 5 6 7 8 9" -- ${cur} ) )
            return 0
            ;;
        -n|--net)
            _filedir
            return 0
            ;;
    esac



    # relevant options to complete
    if [[ ${cur} == -* ]] ; then
      COMPREPLY=( $( compgen -W "${opts}"  -- "$cur" ) )
    fi
    
    
    # stolen from _cd function in bash_completion    
    _cd
} &&
complete -o dirnames -F _slam6D slam6D

# Local variables:
# mode: shell-script
# sh-basic-offset: 4
# sh-indent-comment: t
# indent-tabs-mode: nil
# End:
# ex: ts=4 sw=4 et filetype=sh
