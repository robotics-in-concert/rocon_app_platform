function _rocon_app_complete_exe {
    local arg opts
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]]; then
        opts="add-repo compat index info install list list-repos update rawinfo remove-repo"
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    fi
}

complete -F "_rocon_app_complete_exe" "rocon_app"

