/*
 * PredictiveController.cpp
 *
 *  Created on: 02. 12. 2018
 *      Author: Jiri
 */

#include "PredictiveController.h"
#include "Arduino.h"
#include "MatrixMath.h"
#include "MemoryFree.h"

PredictiveController::PredictiveController() {

	_G = 0.0;
	_last_controll_value = 0.0;
	_predict_value = 0.0;
	_controll_value = 0.0;
	_N2 = 0;
	_nA = 0;
	_nB = 0;
	_nC = 0;
	_alfa = 0.0;
	_last_setPoint=0.0;

	for (int i = 0; i < 8; ++i) {
		_K[i] = 0.0;
		_Kfp[i] = 0.0;
		_setPoint[i] = 0.0;
	}

	for (int i = 0; i < 8; ++i) {
		_Fp[i] = 0.0;
		_Xp[i] = 0.0;
	}

}

void PredictiveController::initialize(float a[], float b[], float c[], int N2,
		int Nu, float q,float alfa, int nA, int nB, int nC) {
	float mat[nA + 2];
	float conv[2] = { 1.0, -1.0 };
//priprava vektoru pro nasobeni polynomu
	for (int i = 0; i < nA + 2; ++i) {
		mat[i] = 0;
	}
//Priprava matic Ap,Bp,Am,Bm,Cm

	/*float App[N2][N2];
	float Bpp[N2][N2];*/
	mtx_type Ap[N2][N2];
	mtx_type Bp[N2][N2];
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < N2; ++j) {
			Ap[i][j] = 0;
			Bp[i][j] = 0;
		}
	}

	float Amp[N2][nA];
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < nA; ++j) {
			Amp[i][j] = 0;
		}
	}

	float Bmp[N2][nB-2];
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < nB-2; ++j) {
			Bmp[i][j] = 0;
		}
	}

	float Cmp[N2][nC-1];
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < nC-1; ++j) {
			Cmp[i][j] = 0;
		}
	}

	float Fpop[N2][nA + nB + nC-3];
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < nA + nB + nC-3; ++j) {
			Fpop[i][j] = 0;
		}
	}

//násobení polynomù
	for (int i = 0; i < nA; ++i) {
		for (int j = 0; j < 2; ++j) {
			mat[i + j] = mat[i + j] + a[i] * conv[j];
		}
	}
//App
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < N2; ++j) {
			if ((i + j) >= N2) {
				break;
			}
			if (j < (nA + 1)) {
				Ap[i + j][i] = mat[j];
			}

		}
	}

//Bpp
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < N2; ++j) {
			if ((i + j) >= (N2)) {
				break;
			}
			if (j < (nB - 1)) {
				Bp[i + j][i] = b[j + 1];
			}

		}
	}


//Amp
	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < nA; ++j) {
			if ((i + j) >= (nA + 1)) {
				break;
			}
			Amp[i][j] = -mat[j + i + 1];

		}
	}
//Bmp
	if (nA > 2) {
		for (int i = 0; i < N2; ++i) {
			for (int j = 0; j < nB - 2; ++j) {
				if ((i + j) >= (nB - 2)) {
					break;
				}
				Bmp[i][j] = b[j + i + 2];
			}
		}
	}
//Cmp
	if (nC > 1) {
		for (int i = 0; i < N2; ++i) {
			for (int j = 0; j < nC - 1; ++j) {
				if ((i + j) >= (nC - 1)) {
					break;
				}
				Cmp[i][j] = c[j + i + 1];
			}
		}

	}
//naplnìní matice AM,BM,CM
	for (int i = 0; i < N2; ++i) {
	            for (int j = 0; j < nA; ++j) {
	                Fpop[i][j] = Amp[i][j];
	            }
	        }
	        if (nC > 1 && nA < 3) {

	            for (int i = 0; i < N2; ++i) {
	                for (int j = nA; j < nA +nC - 1; ++j) {
	                    Fpop[i][j] = Cmp[i][j - nA];
	                }
	            }
	        }
	        if (nA > 2 && nC < 2) {

	            for (int i = 0; i < N2; ++i) {
	                for (int j = nA; j < nA + nB-2; ++j) {
	                    Fpop[i][j] = Bmp[i][j - nA];
	                }
	            }
	        }
	        if (nA > 2 && nC > 1) {

	            for (int i = 0; i < N2; ++i) {
	                for (int j = nA; j < nA + nB-2; ++j) {
	                    Fpop[i][j] = Bmp[i][j-nA];
	                }
	            }

	            for (int i = 0; i < N2; ++i) {
	                for (int j = nA+nB-2; j < nA + nB + nC-3; ++j) {
	                    Fpop[i][j] = Cmp[i][j-nA-nB+2];
	                }
	            }

	        }

	/*
	 for (int i = 0; i < nA+2; ++i) {
	 Serial.print(mat[i], 4);
	 Serial.print(" ");
	 }
	 Serial.println();
	 Serial.println("AP");
	 for (int i = 0; i < N2; ++i) {
	 for (int j = 0; j < N2; ++j) {
	 Serial.print(App[i][j]);
	 Serial.print(" ");

	 }
	 Serial.println();
	 }

	 Serial.println("BP");
	 for (int i = 0; i < N2; ++i) {
	 for (int j = 0; j < N2; ++j) {
	 Serial.print(Bpp[i][j]);
	 Serial.print(" ");

	 }
	 Serial.println();
	 }

	 Serial.println("AM");
	 for (int i = 0; i < N2; ++i) {
	 for (int j = 0; j < nA; ++j) {
	 Serial.print(Amp[i][j]);
	 Serial.print(" ");

	 }
	 Serial.println();
	 }

	 Serial.println("BM");
	 for (int i = 0; i < N2; ++i) {
	 for (int j = 0; j < nB; ++j) {
	 Serial.print(Bmp[i][j]);
	 Serial.print(" ");

	 }
	 Serial.println();
	 }

	 Serial.println("CM");
	 for (int i = 0; i < N2; ++i) {
	 for (int j = 0; j < nC; ++j) {
	 Serial.print(Cmp[i][j]);
	 Serial.print(" ");

	 }
	 Serial.println();
	 }

	 Serial.println("AM CM BM");
	 for (int i = 0; i < N2; ++i) {
	 for (int j = 0; j < nA+nB+nC-3; ++j) {
	 Serial.print(Fpop[i][j]);
	 Serial.print(" ");

	 }
	 Serial.println();
	 }*/

	/*mtx_type Ap[N2][N2];
	mtx_type Bp[N2][N2];*/
	mtx_type Go[N2][N2];
	mtx_type G[N2][Nu];
	mtx_type Fp[N2][nA + nB + nC - 3];
	mtx_type FyFuFe[N2][nA + nB + nC - 3];
	mtx_type R[N2][N2];
	mtx_type Q[Nu][Nu];
	mtx_type Gt[Nu][N2];
	mtx_type L[Nu][N2];

	mtx_type GtR[Nu][N2];
	mtx_type GtRG[Nu][Nu];
	mtx_type GtRGQ[Nu][Nu];
	mtx_type GtRGQGt[Nu][N2];
	mtx_type Lfp[Nu][nA + nB + nC - 3];

	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < N2; ++j) {
			if (i == j) {
				R[i][j] = 1.0f;
			} else {
				R[i][j] = 0.0f;
			}
		}
	}
	for (int i = 0; i < Nu; ++i) {
		for (int j = 0; j < Nu; ++j) {
			if (i == j) {
				Q[i][j] = q;
			} else {
				Q[i][j] = 0.0f;
			}
		}
	}

	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < nA + nB + nC - 3; ++j) {
			FyFuFe[i][j] = Fpop[i][j];
		}
	}

//Matrix.Print((mtx_type*)Ap, N2, N2, "Ap");
//Matrix.Print((mtx_type*)Bp, N2, N2, "Bp");
//Matrix.Print((mtx_type*)FyFuFe, N2, nA+nB+nC-3, "FyFuFe");

	Matrix.Invert((mtx_type*) Ap, N2);

//Matrix.Print((mtx_type*)Ap, N2, N2, "Ap");

	Matrix.Multiply((mtx_type*) Ap, (mtx_type*) Bp, N2, N2, N2, (mtx_type*) Go);

//Matrix.Print((mtx_type*)Go, N2, N2, "Go");

	Matrix.Multiply((mtx_type*) Ap, (mtx_type*) FyFuFe, N2, N2,
			nA + nB + nC - 3, (mtx_type*) Fp);

//Matrix.Print((mtx_type*)Fp, N2, nA+nB+nC-3, "Fp");

	for (int i = 0; i < N2; ++i) {
		for (int j = 0; j < Nu; ++j) {
			G[i][j] = Go[i][j];
		}
	}

//Matrix.Print((mtx_type*)G, N2, Nu, "G");
//Matrix.Print((mtx_type*)R, N2, N2, "R");
//Matrix.Print((mtx_type*)Q, Nu, Nu, "Q");

	Matrix.Transpose((mtx_type*) G, N2, Nu, (mtx_type*) Gt);

//Matrix.Print((mtx_type*)Gt, Nu, N2, "Gt");

	Matrix.Multiply((mtx_type*) Gt, (mtx_type*) R, Nu, N2, N2, (mtx_type*) GtR);
//Matrix.Print((mtx_type*)GtR, Nu, N2, "GtR");
	Matrix.Multiply((mtx_type*) GtR, (mtx_type*) G, Nu, N2, Nu,
			(mtx_type*) GtRG);
//Matrix.Print((mtx_type*)GtRG, Nu, Nu, "GtRG");
	Matrix.Add((mtx_type*) GtRG, (mtx_type*) Q, Nu, Nu, (mtx_type*) GtRGQ);

//Matrix.Print((mtx_type*)GtRGQ, Nu, Nu, "GtRGQ");

	Matrix.Invert((mtx_type*) GtRGQ, Nu);

	Matrix.Multiply((mtx_type*) GtRGQ, (mtx_type*) Gt, Nu, Nu, N2,
			(mtx_type*) GtRGQGt);

//Matrix.Print((mtx_type*)GtRGQGt, Nu, N2, "GtRGQGt");

	Matrix.Multiply((mtx_type*) GtRGQGt, (mtx_type*) R, Nu, N2, N2,
			(mtx_type*) L);

//Matrix.Print((mtx_type*)L, Nu, N2, "L");

	Matrix.Multiply((mtx_type*) L, (mtx_type*) Fp, Nu, N2, nA + nB + nC - 3,
			(mtx_type*) Lfp);

//Matrix.Print((mtx_type*)Lfp, Nu, nA+nB+nC-3, "Lfp");

	_G = G[0][0];

	for (int i = 0; i < N2; ++i) {
		_K[i] = L[0][i];
	}

	for (int i = 0; i < nA + nB + nC - 3; ++i) {
		_Kfp[i] = -Lfp[0][i];
	}

	for (int i = 0; i < nA + nB + nC - 3; ++i) {
		_Fp[i] = Fp[0][i];
	}

	_N2 = N2;
	_nA = nA;
	_nB = nB;
	_nC = nC;

	_alfa = alfa;

	Serial.println(freeMemory());
}

void PredictiveController::showControllerData() {
	Serial.print("G: ");
	Serial.println(_G);

	Serial.print("K: ");
	for (int i = 0; i < _N2; ++i) {
		Serial.print(_K[i]);
		Serial.print(" ");
	}

	Serial.println();
	Serial.print("Kfp: ");
	for (int i = 0; i < _nA + _nB + _nC - 3; ++i) {
		Serial.print(_Kfp[i]);
		Serial.print(" ");
	}

	Serial.println();
	Serial.print("Fp: ");
	for (int i = 0; i < _nA + _nB + _nC - 3; ++i) {
		Serial.print(_Fp[i]);
		Serial.print(" ");
	}
}

void PredictiveController::setSetpoint(float w, long current_time,
		long start_time, long end_time) {
	if (_setPoint[0]!=w && (current_time >= start_time * 1000) && (current_time <= end_time * 1000)) {
		for (int i = 0; i < _N2; ++i) {
			_setPoint[i]=_setPoint[i+1];
		}
		_setPoint[_N2] = _alfa*_last_setPoint+(1-_alfa)*w;
		_last_setPoint=_setPoint[_N2];
	}


}

void PredictiveController::printData(long current_time) {
	Serial.print(current_time / 1000);
	Serial.print(", ");
	Serial.print(_Xp[0]);
	Serial.print(", ");
	Serial.print(_controll_value);
	Serial.print(", ");
	Serial.print(_setPoint[0]);
	Serial.print(", ");
	Serial.print(_predict_value);
	Serial.println();

}

float PredictiveController::process(float last_sample) {
	float deltaControllValue = 0.0;
	float KFpXp = 0.0;
	float FpXp = 0.0;



	for (int i = _nA; i > 0; --i) {
		_Xp[i] = _Xp[i - 1];
	}

	_Xp[0] = last_sample;


	if (_nA > 2 && _nC > 1) {
		for (int i = _nA + _nB - 2; i > _nA; --i) {
			_Xp[i] = _Xp[i - 1];
		}
		_Xp[_nA] = _last_controll_value;

		for (int i = _nA + _nB + _nC - 3; i > _nA + _nB - 2; --i) {
			_Xp[i] = _Xp[i - 1];
		}
		_Xp[_nA + _nB - 2] = last_sample - _predict_value;

		for (int i = 0; i < _nA + _nB + _nC - 3; ++i) {
			KFpXp += _Kfp[i] * _Xp[i];
		}

		for (int i = 0; i < _nA + _nB + _nC - 3; ++i) {
			FpXp += _Fp[i] * _Xp[i];
		}

	}

	if (_nA < 3 && _nC > 1) {
		for (int i = _nA + _nC - 1; i > _nA; --i) {
			_Xp[i] = _Xp[i - 1];
		}
		_Xp[_nA] = last_sample - _predict_value;

		for (int i = 0; i < _nA + _nC - 1; ++i) {
			KFpXp += _Kfp[i] * _Xp[i];
		}

		for (int i = 0; i < _nA + _nC - 1; ++i) {
			FpXp += _Fp[i] * _Xp[i];
		}
	}

	for (int i = 0; i < _N2; ++i) {
		deltaControllValue += _K[i] * _setPoint[i];
	}

	deltaControllValue += KFpXp;

	_controll_value += deltaControllValue;

	_last_controll_value = deltaControllValue;

	_predict_value = _G * deltaControllValue + FpXp;

	if (_controll_value < 0.0) {
		_controll_value = 0.0;
	}
	if (_controll_value > 5.0) {
		_controll_value = 5.0;
	}

	return _controll_value;
}

PredictiveController::~PredictiveController() {
	// TODO Auto-generated destructor stub
}

