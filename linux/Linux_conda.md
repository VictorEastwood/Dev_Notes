# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/yang/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/yang/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/yang/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/yang/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup

# 强制不激活基础环境
conda deactivate 2>/dev/null
# <<< conda initialize <<<
