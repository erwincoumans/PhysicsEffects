/*
   Copyright (C) 2009 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#pragma once

#include "App/PfxApp.h"

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;

namespace PfxViewer {

	/// <summary>
	/// WorldPropertyDlg の概要
	///
	/// 警告: このクラスの名前を変更する場合、このクラスが依存するすべての .resx ファイルに関連付けられた
	///          マネージ リソース コンパイラ ツールに対して 'Resource File Name' プロパティを
	///          変更する必要があります。この変更を行わないと、
	///          デザイナと、このフォームに関連付けられたローカライズ済みリソースとが、
	///          正しく相互に利用できなくなります。
	/// </summary>
	public ref class WorldPropertyDlg : public System::Windows::Forms::Form
	{
	public:
		PfxApp::PfxAppWorldProperty *mWorldProperty;

		WorldPropertyDlg(void)
		{
			InitializeComponent();
			//
			//TODO: ここにコンストラクタ コードを追加します
			//
		}

	protected:
		/// <summary>
		/// 使用中のリソースをすべてクリーンアップします。
		/// </summary>
		~WorldPropertyDlg()
		{
			if (components)
			{
				delete components;
			}
		}

	private: System::Windows::Forms::Button^  btnOK;
	protected: 

	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::CheckBox^  chkboxSleepEnable;
	private: System::Windows::Forms::TextBox^  editRigMemory;

	private: System::Windows::Forms::TextBox^  editBufferSize;
	private: System::Windows::Forms::TextBox^  editMaxContactPairs;
	private: System::Windows::Forms::TextBox^  editMaxJoints;
	private: System::Windows::Forms::TextBox^  editMaxBodies;
	private: System::Windows::Forms::TextBox^  editMaxInstances;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::TextBox^  editRayMemory;

	private: System::Windows::Forms::NumericUpDown^  numUpDownJointIteration;
	private: System::Windows::Forms::NumericUpDown^  numUpDownContactIteration;
	private: System::Windows::Forms::NumericUpDown^  numUpDownAlloc;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::TextBox^  editTotalMemory;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::TrackBar^  barPick;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::NumericUpDown^  numUpDownSubStep;
	private: System::Windows::Forms::Label^  label13;
	private: System::Windows::Forms::NumericUpDown^  numUpDownFrameRate;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::Label^  label16;
	private: System::Windows::Forms::TrackBar^  barLineWidth;



	protected: 

	protected: 

	private:
		/// <summary>
		/// 必要なデザイナ変数です。
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// デザイナ サポートに必要なメソッドです。このメソッドの内容を
		/// コード エディタで変更しないでください。
		/// </summary>
		void InitializeComponent(void)
		{
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(WorldPropertyDlg::typeid));
			this->btnOK = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->editTotalMemory = (gcnew System::Windows::Forms::TextBox());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->editRayMemory = (gcnew System::Windows::Forms::TextBox());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->numUpDownAlloc = (gcnew System::Windows::Forms::NumericUpDown());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->editRigMemory = (gcnew System::Windows::Forms::TextBox());
			this->editMaxContactPairs = (gcnew System::Windows::Forms::TextBox());
			this->editMaxJoints = (gcnew System::Windows::Forms::TextBox());
			this->editMaxBodies = (gcnew System::Windows::Forms::TextBox());
			this->editMaxInstances = (gcnew System::Windows::Forms::TextBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->numUpDownFrameRate = (gcnew System::Windows::Forms::NumericUpDown());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->numUpDownSubStep = (gcnew System::Windows::Forms::NumericUpDown());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->numUpDownJointIteration = (gcnew System::Windows::Forms::NumericUpDown());
			this->numUpDownContactIteration = (gcnew System::Windows::Forms::NumericUpDown());
			this->chkboxSleepEnable = (gcnew System::Windows::Forms::CheckBox());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->barLineWidth = (gcnew System::Windows::Forms::TrackBar());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->barPick = (gcnew System::Windows::Forms::TrackBar());
			this->groupBox1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownAlloc))->BeginInit();
			this->groupBox3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownFrameRate))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownSubStep))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownJointIteration))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownContactIteration))->BeginInit();
			this->groupBox2->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->barLineWidth))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->barPick))->BeginInit();
			this->SuspendLayout();
			// 
			// btnOK
			// 
			this->btnOK->Location = System::Drawing::Point(248, 448);
			this->btnOK->Name = L"btnOK";
			this->btnOK->Size = System::Drawing::Size(83, 27);
			this->btnOK->TabIndex = 1;
			this->btnOK->Text = L"OK";
			this->btnOK->UseVisualStyleBackColor = true;
			this->btnOK->Click += gcnew System::EventHandler(this, &WorldPropertyDlg::btnOK_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->label11);
			this->groupBox1->Controls->Add(this->editTotalMemory);
			this->groupBox1->Controls->Add(this->label8);
			this->groupBox1->Controls->Add(this->editRayMemory);
			this->groupBox1->Controls->Add(this->label7);
			this->groupBox1->Controls->Add(this->numUpDownAlloc);
			this->groupBox1->Controls->Add(this->label6);
			this->groupBox1->Controls->Add(this->editRigMemory);
			this->groupBox1->Controls->Add(this->editMaxContactPairs);
			this->groupBox1->Controls->Add(this->editMaxJoints);
			this->groupBox1->Controls->Add(this->editMaxBodies);
			this->groupBox1->Controls->Add(this->editMaxInstances);
			this->groupBox1->Controls->Add(this->label5);
			this->groupBox1->Controls->Add(this->label4);
			this->groupBox1->Controls->Add(this->label3);
			this->groupBox1->Controls->Add(this->label2);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Location = System::Drawing::Point(8, 8);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(320, 208);
			this->groupBox1->TabIndex = 7;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Buffer";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(8, 160);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(133, 12);
			this->label11->TabIndex = 23;
			this->label11->Text = L"Total Requested Memory";
			// 
			// editTotalMemory
			// 
			this->editTotalMemory->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editTotalMemory->Location = System::Drawing::Point(176, 160);
			this->editTotalMemory->Name = L"editTotalMemory";
			this->editTotalMemory->ReadOnly = true;
			this->editTotalMemory->Size = System::Drawing::Size(128, 19);
			this->editTotalMemory->TabIndex = 22;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(264, 184);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(44, 12);
			this->label8->TabIndex = 21;
			this->label8->Text = L"MBytes";
			// 
			// editRayMemory
			// 
			this->editRayMemory->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editRayMemory->Location = System::Drawing::Point(176, 136);
			this->editRayMemory->Name = L"editRayMemory";
			this->editRayMemory->ReadOnly = true;
			this->editRayMemory->Size = System::Drawing::Size(128, 19);
			this->editRayMemory->TabIndex = 20;
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(8, 136);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(151, 12);
			this->label7->TabIndex = 19;
			this->label7->Text = L"Requested RayCast Memory";
			// 
			// numUpDownAlloc
			// 
			this->numUpDownAlloc->Enabled = false;
			this->numUpDownAlloc->Location = System::Drawing::Point(176, 184);
			this->numUpDownAlloc->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) {1024, 0, 0, 0});
			this->numUpDownAlloc->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) {1, 0, 0, 0});
			this->numUpDownAlloc->Name = L"numUpDownAlloc";
			this->numUpDownAlloc->Size = System::Drawing::Size(80, 19);
			this->numUpDownAlloc->TabIndex = 13;
			this->numUpDownAlloc->ThousandsSeparator = true;
			this->numUpDownAlloc->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) {100, 0, 0, 0});
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(8, 184);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(97, 12);
			this->label6->TabIndex = 18;
			this->label6->Text = L"Allocated Memory";
			// 
			// editRigMemory
			// 
			this->editRigMemory->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editRigMemory->Location = System::Drawing::Point(176, 112);
			this->editRigMemory->Name = L"editRigMemory";
			this->editRigMemory->ReadOnly = true;
			this->editRigMemory->Size = System::Drawing::Size(128, 19);
			this->editRigMemory->TabIndex = 17;
			// 
			// editMaxContactPairs
			// 
			this->editMaxContactPairs->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editMaxContactPairs->Location = System::Drawing::Point(176, 88);
			this->editMaxContactPairs->Name = L"editMaxContactPairs";
			this->editMaxContactPairs->ReadOnly = true;
			this->editMaxContactPairs->Size = System::Drawing::Size(128, 19);
			this->editMaxContactPairs->TabIndex = 16;
			// 
			// editMaxJoints
			// 
			this->editMaxJoints->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editMaxJoints->Location = System::Drawing::Point(176, 64);
			this->editMaxJoints->Name = L"editMaxJoints";
			this->editMaxJoints->ReadOnly = true;
			this->editMaxJoints->Size = System::Drawing::Size(128, 19);
			this->editMaxJoints->TabIndex = 15;
			// 
			// editMaxBodies
			// 
			this->editMaxBodies->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editMaxBodies->Location = System::Drawing::Point(176, 40);
			this->editMaxBodies->Name = L"editMaxBodies";
			this->editMaxBodies->ReadOnly = true;
			this->editMaxBodies->Size = System::Drawing::Size(128, 19);
			this->editMaxBodies->TabIndex = 14;
			// 
			// editMaxInstances
			// 
			this->editMaxInstances->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->editMaxInstances->Location = System::Drawing::Point(176, 16);
			this->editMaxInstances->Name = L"editMaxInstances";
			this->editMaxInstances->ReadOnly = true;
			this->editMaxInstances->Size = System::Drawing::Size(128, 19);
			this->editMaxInstances->TabIndex = 13;
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(8, 112);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(159, 12);
			this->label5->TabIndex = 12;
			this->label5->Text = L"Requested RigidBody Memory";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(8, 88);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(100, 12);
			this->label4->TabIndex = 11;
			this->label4->Text = L"Max Contact Pairs";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(8, 64);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(62, 12);
			this->label3->TabIndex = 10;
			this->label3->Text = L"Max Joints";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(8, 40);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(95, 12);
			this->label2->TabIndex = 9;
			this->label2->Text = L"Max Rigid Bodies";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(8, 16);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(79, 12);
			this->label1->TabIndex = 8;
			this->label1->Text = L"Max Instances";
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->label15);
			this->groupBox3->Controls->Add(this->numUpDownFrameRate);
			this->groupBox3->Controls->Add(this->label14);
			this->groupBox3->Controls->Add(this->numUpDownSubStep);
			this->groupBox3->Controls->Add(this->label13);
			this->groupBox3->Controls->Add(this->numUpDownJointIteration);
			this->groupBox3->Controls->Add(this->numUpDownContactIteration);
			this->groupBox3->Controls->Add(this->chkboxSleepEnable);
			this->groupBox3->Controls->Add(this->label9);
			this->groupBox3->Controls->Add(this->label10);
			this->groupBox3->Location = System::Drawing::Point(8, 224);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(320, 136);
			this->groupBox3->TabIndex = 13;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Simulation";
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(224, 16);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(58, 12);
			this->label15->TabIndex = 17;
			this->label15->Text = L"frame/sec";
			// 
			// numUpDownFrameRate
			// 
			this->numUpDownFrameRate->Increment = System::Decimal(gcnew cli::array< System::Int32 >(4) {5, 0, 0, 0});
			this->numUpDownFrameRate->Location = System::Drawing::Point(152, 16);
			this->numUpDownFrameRate->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) {240, 0, 0, 0});
			this->numUpDownFrameRate->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) {15, 0, 0, 0});
			this->numUpDownFrameRate->Name = L"numUpDownFrameRate";
			this->numUpDownFrameRate->Size = System::Drawing::Size(64, 19);
			this->numUpDownFrameRate->TabIndex = 16;
			this->numUpDownFrameRate->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) {60, 0, 0, 0});
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(8, 16);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(65, 12);
			this->label14->TabIndex = 15;
			this->label14->Text = L"Frame Rate";
			// 
			// numUpDownSubStep
			// 
			this->numUpDownSubStep->Location = System::Drawing::Point(152, 40);
			this->numUpDownSubStep->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) {10, 0, 0, 0});
			this->numUpDownSubStep->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) {1, 0, 0, 0});
			this->numUpDownSubStep->Name = L"numUpDownSubStep";
			this->numUpDownSubStep->Size = System::Drawing::Size(64, 19);
			this->numUpDownSubStep->TabIndex = 14;
			this->numUpDownSubStep->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) {1, 0, 0, 0});
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(8, 40);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(51, 12);
			this->label13->TabIndex = 13;
			this->label13->Text = L"Sub Step";
			// 
			// numUpDownJointIteration
			// 
			this->numUpDownJointIteration->Location = System::Drawing::Point(152, 88);
			this->numUpDownJointIteration->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) {255, 0, 0, 0});
			this->numUpDownJointIteration->Name = L"numUpDownJointIteration";
			this->numUpDownJointIteration->Size = System::Drawing::Size(64, 19);
			this->numUpDownJointIteration->TabIndex = 12;
			// 
			// numUpDownContactIteration
			// 
			this->numUpDownContactIteration->Location = System::Drawing::Point(152, 64);
			this->numUpDownContactIteration->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) {255, 0, 0, 0});
			this->numUpDownContactIteration->Name = L"numUpDownContactIteration";
			this->numUpDownContactIteration->Size = System::Drawing::Size(64, 19);
			this->numUpDownContactIteration->TabIndex = 11;
			// 
			// chkboxSleepEnable
			// 
			this->chkboxSleepEnable->AutoSize = true;
			this->chkboxSleepEnable->Location = System::Drawing::Point(8, 112);
			this->chkboxSleepEnable->Name = L"chkboxSleepEnable";
			this->chkboxSleepEnable->Size = System::Drawing::Size(90, 16);
			this->chkboxSleepEnable->TabIndex = 10;
			this->chkboxSleepEnable->Text = L"Sleep Enable";
			this->chkboxSleepEnable->UseVisualStyleBackColor = true;
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(8, 88);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(113, 12);
			this->label9->TabIndex = 9;
			this->label9->Text = L"Joint Solver Iteration";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(8, 64);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(127, 12);
			this->label10->TabIndex = 8;
			this->label10->Text = L"Contact Solver Iteration";
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->label16);
			this->groupBox2->Controls->Add(this->barLineWidth);
			this->groupBox2->Controls->Add(this->label12);
			this->groupBox2->Controls->Add(this->barPick);
			this->groupBox2->Location = System::Drawing::Point(8, 368);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(320, 72);
			this->groupBox2->TabIndex = 14;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Config";
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(8, 40);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(58, 12);
			this->label16->TabIndex = 26;
			this->label16->Text = L"Line Width";
			// 
			// barLineWidth
			// 
			this->barLineWidth->AutoSize = false;
			this->barLineWidth->Location = System::Drawing::Point(80, 40);
			this->barLineWidth->Maximum = 20;
			this->barLineWidth->Minimum = 1;
			this->barLineWidth->Name = L"barLineWidth";
			this->barLineWidth->Size = System::Drawing::Size(232, 24);
			this->barLineWidth->TabIndex = 25;
			this->barLineWidth->TickFrequency = 10;
			this->barLineWidth->TickStyle = System::Windows::Forms::TickStyle::None;
			this->barLineWidth->Value = 5;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(8, 16);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(62, 12);
			this->label12->TabIndex = 24;
			this->label12->Text = L"Pick Power";
			// 
			// barPick
			// 
			this->barPick->AutoSize = false;
			this->barPick->Location = System::Drawing::Point(80, 16);
			this->barPick->Maximum = 100;
			this->barPick->Name = L"barPick";
			this->barPick->Size = System::Drawing::Size(232, 24);
			this->barPick->TabIndex = 0;
			this->barPick->TickFrequency = 10;
			this->barPick->TickStyle = System::Windows::Forms::TickStyle::None;
			this->barPick->Value = 10;
			// 
			// WorldPropertyDlg
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(336, 482);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->btnOK);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedDialog;
			this->Icon = (cli::safe_cast<System::Drawing::Icon^  >(resources->GetObject(L"$this.Icon")));
			this->MaximizeBox = false;
			this->MinimizeBox = false;
			this->Name = L"WorldPropertyDlg";
			this->ShowIcon = false;
			this->StartPosition = System::Windows::Forms::FormStartPosition::CenterParent;
			this->Text = L"World Property";
			this->Load += gcnew System::EventHandler(this, &WorldPropertyDlg::WorldPropertyDlg_Load);
			this->FormClosed += gcnew System::Windows::Forms::FormClosedEventHandler(this, &WorldPropertyDlg::WorldPropertyDlg_FormClosed);
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownAlloc))->EndInit();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownFrameRate))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownSubStep))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownJointIteration))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numUpDownContactIteration))->EndInit();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->barLineWidth))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->barPick))->EndInit();
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void btnOK_Click(System::Object^  sender, System::EventArgs^  e) {
				 this->Close();
			 }
private: System::Void WorldPropertyDlg_Load(System::Object^  sender, System::EventArgs^  e) {
			 editMaxInstances->Text = "" + mWorldProperty->maxInstances;
			 editMaxBodies->Text = "" + mWorldProperty->maxDynBodies;
			 editMaxJoints->Text = "" + mWorldProperty->maxJoints;
			 editMaxContactPairs->Text = "" + mWorldProperty->maxContactPairs;
			 editRigMemory->Text = "" + mWorldProperty->rigBufferSize + " bytes";
			 editRayMemory->Text = "" + mWorldProperty->rayBufferSize + " bytes";
			 editTotalMemory->Text = "" + (mWorldProperty->rigBufferSize + mWorldProperty->rayBufferSize) + " bytes";
			 numUpDownFrameRate->Value = System::Decimal((int)mWorldProperty->frameRate);
			 numUpDownSubStep->Value = System::Decimal((int)mWorldProperty->subStepCount);
			 numUpDownContactIteration->Value = System::Decimal((int)mWorldProperty->contactIteration);
			 numUpDownJointIteration->Value = System::Decimal((int)mWorldProperty->jointIteration);
			 numUpDownAlloc->Value = System::Decimal((int)mWorldProperty->allocatedBuffSize);
			 chkboxSleepEnable->Checked = mWorldProperty->sleepEnable;
			 barPick->Value = (int)mWorldProperty->pickPower;
			 barLineWidth->Value = (int)mWorldProperty->lineWidth;
		 }
private: System::Void WorldPropertyDlg_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e) {
			 mWorldProperty->frameRate = (uint32_t)Decimal::ToInt32(numUpDownFrameRate->Value);
			 mWorldProperty->subStepCount = (uint32_t)Decimal::ToInt32(numUpDownSubStep->Value);
			 mWorldProperty->contactIteration = (uint32_t)Decimal::ToInt32(numUpDownContactIteration->Value);
			 mWorldProperty->jointIteration = (uint32_t)Decimal::ToInt32(numUpDownJointIteration->Value);
			 mWorldProperty->allocatedBuffSize = (uint32_t)Decimal::ToInt32(numUpDownAlloc->Value);
			 mWorldProperty->sleepEnable = chkboxSleepEnable->Checked;
			 mWorldProperty->pickPower = barPick->Value;
			 mWorldProperty->lineWidth = barLineWidth->Value;
		 }
};
}
