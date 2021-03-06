/*!
 * \author Nicolas Van der Noot
 * \file user_realtime_visu.c
 * \brief Java visualization configuration
 *
 * In order to use C++ features, you just need to change the extension of this file (.c) to .cc (or .cpp).
 */

#ifdef VISU_3D

#include "user_realtime.h"
#include "mbs_data.h"

/*! \brief Java visualization user configuration
 * 
 * \param[in] mbs_data Robotran main structure
 * \param[in] nb_models number of models (.mbs files) loaded for visualization
 * \param[in] nb_q number of joints in the .mbs file (for each model)
 * \param[out] q_vec vector to fill with the positions (for each model)
 *
 * Fill q_vec[i][j] with the corresponding values for the joints ([rad] for rotational joints,
 *      [m] for translational joints). j-1 is the index of the joint j indicated in the corresponding 
 *      .mbs file used for the visualization (pay attention to the -1 !). If you want to know the number
 *      of joints to fill for model 'i', print 'nb_q[i]'. Do not use indexes below '0' or above 'nb_q[i]-1'.
 *
 * This file is initially configured to work with the .mbs file located in dataR.
 * Modifying this file is only useful if you want to display something else than
 * the model used to compute the dynamics with Robotran (or if you want to combine different models).
 * For instance, it can be used to display extra bodies (like moving arrows)
 * on top of the ones used by Robotran dynamics.
 * In this case, create one (or more) new .mbs file (on top of the one in dataR) only used for visualization.
 * Place it somewhere in your project and indicate its path in 'user_realtime_options.c' as
 * 'options->mbs_file[i] = NEW_PATH_WITH_MBS_FILE;'. You must also adapt 'options->nb_q[i]'.
 * Finally, modify the 'user_realtime_visu' function accordingly.
 */
void user_realtime_visu(MbsData* mbs_data, int nb_models, int *nb_q, double **q_vec)
{
    int i, j;

    for(i=0; i<nb_models; i++)
    {
        for(j=1; j<=nb_q[i]; j++)
        {
            q_vec[i][j-1] = i ? 0.0 : mbs_data->q[j];
        }
    }
}

#endif
