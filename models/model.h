#pragma once
#include <cstdarg>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class RandomForest {
                public:
                    /**
                    * Predict class for features vector
                    */
                    int predict(float *x) {
                        uint8_t votes[4] = { 0 };
                        // tree #1
                        if (x[1] <= 0.75692018866539) {
                            if (x[0] <= 0.2857782170176506) {
                                votes[2] += 1;
                            }

                            else {
                                if (x[0] <= 1.016722321510315) {
                                    if (x[0] <= 0.7551084756851196) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        if (x[1] <= -0.5465880744159222) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }
                                }

                                else {
                                    if (x[3] <= 1.3552724123001099) {
                                        if (x[0] <= 1.0379178524017334) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[1] += 1;
                                        }
                                    }

                                    else {
                                        if (x[2] <= -0.49996744096279144) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }
                                }
                            }
                        }

                        else {
                            votes[3] += 1;
                        }

                        // tree #2
                        if (x[2] <= 0.5304074883460999) {
                            if (x[3] <= -0.6917167007923126) {
                                if (x[0] <= 0.4541315361857414) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[1] += 1;
                                }
                            }

                            else {
                                if (x[2] <= -0.1985463872551918) {
                                    if (x[0] <= 0.28153911605477333) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        if (x[0] <= 0.781148761510849) {
                                            votes[0] += 1;
                                        }

                                        else {
                                            votes[1] += 1;
                                        }
                                    }
                                }

                                else {
                                    if (x[1] <= -0.10716640576720238) {
                                        votes[2] += 1;
                                    }

                                    else {
                                        if (x[3] <= -0.5267366468906403) {
                                            votes[0] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }
                                }
                            }
                        }

                        else {
                            votes[3] += 1;
                        }

                        // tree #3
                        if (x[0] <= -0.9393251836299896) {
                            votes[3] += 1;
                        }

                        else {
                            if (x[1] <= -1.0253609120845795) {
                                votes[1] += 1;
                            }

                            else {
                                if (x[1] <= -0.06617559120059013) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }
                        }

                        // tree #4
                        if (x[1] <= -1.0253609120845795) {
                            votes[1] += 1;
                        }

                        else {
                            if (x[0] <= 0.2857782170176506) {
                                if (x[1] <= 0.2765077129006386) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[3] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        // tree #5
                        if (x[1] <= -1.0319194495677948) {
                            votes[1] += 1;
                        }

                        else {
                            if (x[2] <= 0.5304074883460999) {
                                if (x[1] <= -0.06617559120059013) {
                                    votes[2] += 1;
                                }

                                else {
                                    if (x[1] <= 0.773316502571106) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }
                            }

                            else {
                                votes[3] += 1;
                            }
                        }

                        // tree #6
                        if (x[2] <= 0.5418407022953033) {
                            if (x[0] <= 0.2857782170176506) {
                                if (x[0] <= -0.6946678459644318) {
                                    votes[3] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }

                            else {
                                if (x[0] <= 0.7551084756851196) {
                                    votes[0] += 1;
                                }

                                else {
                                    if (x[1] <= -0.5400295369327068) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }
                            }
                        }

                        else {
                            votes[3] += 1;
                        }

                        // tree #7
                        if (x[1] <= -1.0302797853946686) {
                            votes[1] += 1;
                        }

                        else {
                            if (x[0] <= 0.27427205815911293) {
                                if (x[1] <= 0.2781473472714424) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[3] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        // tree #8
                        if (x[0] <= -0.9532536864280701) {
                            votes[3] += 1;
                        }

                        else {
                            if (x[1] <= -1.0253609120845795) {
                                votes[1] += 1;
                            }

                            else {
                                if (x[1] <= -0.05633779242634773) {
                                    votes[2] += 1;
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }
                        }

                        // tree #9
                        if (x[0] <= -0.9605207145214081) {
                            votes[3] += 1;
                        }

                        else {
                            if (x[1] <= -0.057977426797151566) {
                                if (x[1] <= -1.0319194495677948) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[2] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        // tree #10
                        if (x[1] <= -0.06617559120059013) {
                            if (x[1] <= -1.0302797853946686) {
                                votes[1] += 1;
                            }

                            else {
                                votes[2] += 1;
                            }
                        }

                        else {
                            if (x[2] <= 0.3398539572954178) {
                                votes[0] += 1;
                            }

                            else {
                                votes[3] += 1;
                            }
                        }

                        // return argmax of votes
                        uint8_t classIdx = 0;
                        float maxVotes = votes[0];

                        for (uint8_t i = 1; i < 4; i++) {
                            if (votes[i] > maxVotes) {
                                classIdx = i;
                                maxVotes = votes[i];
                            }
                        }

                        return classIdx;
                    }

                protected:
                };
            }
        }
    }