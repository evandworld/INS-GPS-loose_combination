function qo = qconj(qi)
%四元数求共轭
%conj-Conjugate-共轭
    qo = [qi(1); -qi(2:4)];