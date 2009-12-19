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

#include "Render/GLRender.h"
#include "App/PfxApp.h"
#include "WorldPropertyDlg.h"

namespace PfxViewer {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Form1 の概要
	///
	/// 警告: このクラスの名前を変更する場合、このクラスが依存するすべての .resx ファイルに関連付けられた
	///          マネージ リソース コンパイラ ツールに対して 'Resource File Name' プロパティを
	///          変更する必要があります。この変更を行わないと、
	///          デザイナと、このフォームに関連付けられたローカライズ済みリソースとが、
	///          正しく相互に利用できなくなります。
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
		PfxApp *mApp;
		GLRender *mRender;
		bool mMouseOver;
		int mMouseX,mMouseY;
		DWORD mLastTime;
		DWORD mInterval;

	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::MenuStrip^  menuStrip1;
	private: System::Windows::Forms::ToolStripMenuItem^  menuFile;
	private: System::Windows::Forms::ToolStripMenuItem^  menuFileOpen;
	private: System::Windows::Forms::ToolStripMenuItem^  menuFileSave;
	private: System::Windows::Forms::ToolStripMenuItem^  menuConfig;
	private: System::Windows::Forms::ToolStripMenuItem^  menuConfigWorld;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRender;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderAABB;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderJoint;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderContact;
	private: System::Windows::Forms::CheckBox^  btnSimulate;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderShadow;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderSilhouette;



	private: System::Windows::Forms::TextBox^  textInfo;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderLocalAxis;
	private: System::Windows::Forms::ToolStripMenuItem^  menuFileAdd;



	private: System::Windows::Forms::Button^  btnFrame;

	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator1;
	private: System::Windows::Forms::ToolStripMenuItem^  menuOpenSnapShot;

	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderPerformance;
	private: System::Windows::Forms::ToolStripMenuItem^  menuSaveSnapShot;
	private: System::Windows::Forms::ToolStripMenuItem^  menuRenderMeshIslands;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::RadioButton^  radioModeView;
	private: System::Windows::Forms::RadioButton^  radioModePick;
	private: System::Windows::Forms::ToolStripMenuItem^  menuVSync;
	private: System::Windows::Forms::ToolStripMenuItem^  editToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  menuDeleteSelection;
	private: System::Windows::Forms::ToolTip^  toolTip1;
	private: System::Windows::Forms::ToolStripMenuItem^  menuHelp;

	private: System::Windows::Forms::ToolStripMenuItem^  menuAbout;







	private: System::Windows::Forms::ToolStripMenuItem^  menuReset;

	public:
		Form1(void) : mRender(NULL)
		{
			InitializeComponent();

			mLastTime = 0;
			mInterval = 1.0f/60.0f*1000.0f;

			mRender = new GLRender();
			mRender->setHWND((HWND)pictureBox1->Handle.ToInt32());
			mRender->setWidth(pictureBox1->Width);
			mRender->setHeight(pictureBox1->Height);
			mRender->initializeContext();

			mApp = new PfxApp();
			mApp->onInit(mRender);

			//timer1->Enabled = true;

			mMouseOver = false;
		}

	protected:
		/// <summary>
		/// 使用中のリソースをすべてクリーンアップします。
		/// </summary>
		~Form1()
		{
			mApp->onShutdown();
			mRender->finalizeContext();

			delete mApp;
			delete mRender;

			if (components)
			{
				delete components;
			}
		}

	private: System::Windows::Forms::Panel^  panel1;
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::ComponentModel::IContainer^  components;
	protected: 

	protected: 

	protected: 

	private:
		/// <summary>
		/// 必要なデザイナ変数です。
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// デザイナ サポートに必要なメソッドです。このメソッドの内容を
		/// コード エディタで変更しないでください。
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(Form1::typeid));
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->radioModeView = (gcnew System::Windows::Forms::RadioButton());
			this->radioModePick = (gcnew System::Windows::Forms::RadioButton());
			this->btnFrame = (gcnew System::Windows::Forms::Button());
			this->textInfo = (gcnew System::Windows::Forms::TextBox());
			this->btnSimulate = (gcnew System::Windows::Forms::CheckBox());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
			this->menuFile = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuFileOpen = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuFileSave = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuFileAdd = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuReset = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator1 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->menuOpenSnapShot = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuSaveSnapShot = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->editToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuDeleteSelection = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRender = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderLocalAxis = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderAABB = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderJoint = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderContact = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderMeshIslands = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderShadow = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderSilhouette = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuRenderPerformance = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuVSync = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuConfig = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuConfigWorld = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuHelp = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuAbout = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolTip1 = (gcnew System::Windows::Forms::ToolTip(this->components));
			this->panel1->SuspendLayout();
			this->groupBox1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->BeginInit();
			this->menuStrip1->SuspendLayout();
			this->SuspendLayout();
			// 
			// panel1
			// 
			this->panel1->Controls->Add(this->groupBox1);
			this->panel1->Controls->Add(this->btnFrame);
			this->panel1->Controls->Add(this->textInfo);
			this->panel1->Controls->Add(this->btnSimulate);
			this->panel1->Dock = System::Windows::Forms::DockStyle::Bottom;
			this->panel1->Location = System::Drawing::Point(0, 456);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(592, 89);
			this->panel1->TabIndex = 1;
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->radioModeView);
			this->groupBox1->Controls->Add(this->radioModePick);
			this->groupBox1->Location = System::Drawing::Point(512, 0);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(72, 48);
			this->groupBox1->TabIndex = 8;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Mode";
			// 
			// radioModeView
			// 
			this->radioModeView->Appearance = System::Windows::Forms::Appearance::Button;
			this->radioModeView->BackgroundImage = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"radioModeView.BackgroundImage")));
			this->radioModeView->BackgroundImageLayout = System::Windows::Forms::ImageLayout::Center;
			this->radioModeView->Location = System::Drawing::Point(40, 16);
			this->radioModeView->Name = L"radioModeView";
			this->radioModeView->Size = System::Drawing::Size(24, 24);
			this->radioModeView->TabIndex = 1;
			this->toolTip1->SetToolTip(this->radioModeView, L"Look at");
			this->radioModeView->UseVisualStyleBackColor = true;
			// 
			// radioModePick
			// 
			this->radioModePick->Appearance = System::Windows::Forms::Appearance::Button;
			this->radioModePick->BackgroundImage = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"radioModePick.BackgroundImage")));
			this->radioModePick->BackgroundImageLayout = System::Windows::Forms::ImageLayout::Center;
			this->radioModePick->Checked = true;
			this->radioModePick->Location = System::Drawing::Point(8, 16);
			this->radioModePick->Name = L"radioModePick";
			this->radioModePick->Size = System::Drawing::Size(24, 24);
			this->radioModePick->TabIndex = 0;
			this->radioModePick->TabStop = true;
			this->toolTip1->SetToolTip(this->radioModePick, L"Pick object");
			this->radioModePick->UseVisualStyleBackColor = true;
			// 
			// btnFrame
			// 
			this->btnFrame->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"btnFrame.Image")));
			this->btnFrame->Location = System::Drawing::Point(552, 56);
			this->btnFrame->Name = L"btnFrame";
			this->btnFrame->Size = System::Drawing::Size(32, 24);
			this->btnFrame->TabIndex = 7;
			this->toolTip1->SetToolTip(this->btnFrame, L"Step simulation");
			this->btnFrame->UseVisualStyleBackColor = true;
			this->btnFrame->Click += gcnew System::EventHandler(this, &Form1::btnFrame_Click);
			// 
			// textInfo
			// 
			this->textInfo->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->textInfo->Location = System::Drawing::Point(8, 8);
			this->textInfo->Multiline = true;
			this->textInfo->Name = L"textInfo";
			this->textInfo->ReadOnly = true;
			this->textInfo->Size = System::Drawing::Size(496, 72);
			this->textInfo->TabIndex = 3;
			this->textInfo->TabStop = false;
			// 
			// btnSimulate
			// 
			this->btnSimulate->Appearance = System::Windows::Forms::Appearance::Button;
			this->btnSimulate->CheckAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->btnSimulate->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"btnSimulate.Image")));
			this->btnSimulate->Location = System::Drawing::Point(512, 56);
			this->btnSimulate->Name = L"btnSimulate";
			this->btnSimulate->Size = System::Drawing::Size(32, 24);
			this->btnSimulate->TabIndex = 2;
			this->btnSimulate->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			this->toolTip1->SetToolTip(this->btnSimulate, L"Run simulation");
			this->btnSimulate->UseVisualStyleBackColor = true;
			this->btnSimulate->CheckedChanged += gcnew System::EventHandler(this, &Form1::btnSimulate_CheckedChanged);
			// 
			// pictureBox1
			// 
			this->pictureBox1->AllowDrop = true;
			this->pictureBox1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->pictureBox1->Location = System::Drawing::Point(0, 24);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(592, 432);
			this->pictureBox1->TabIndex = 2;
			this->pictureBox1->TabStop = false;
			this->pictureBox1->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::pictureBox1_MouseMove);
			this->pictureBox1->DragDrop += gcnew System::Windows::Forms::DragEventHandler(this, &Form1::pictureBox1_DragDrop);
			this->pictureBox1->Resize += gcnew System::EventHandler(this, &Form1::pictureBox1_Resize);
			this->pictureBox1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::pictureBox1_MouseDown);
			this->pictureBox1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::pictureBox1_MouseUp);
			this->pictureBox1->DragEnter += gcnew System::Windows::Forms::DragEventHandler(this, &Form1::pictureBox1_DragEnter);
			// 
			// timer1
			// 
			this->timer1->Interval = 1;
			this->timer1->Tick += gcnew System::EventHandler(this, &Form1::timer1_Tick);
			// 
			// menuStrip1
			// 
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(5) {this->menuFile, this->editToolStripMenuItem, 
				this->menuRender, this->menuConfig, this->menuHelp});
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Size = System::Drawing::Size(592, 24);
			this->menuStrip1->TabIndex = 3;
			this->menuStrip1->Text = L"menuStrip1";
			// 
			// menuFile
			// 
			this->menuFile->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(7) {this->menuFileOpen, 
				this->menuFileSave, this->menuFileAdd, this->menuReset, this->toolStripSeparator1, this->menuOpenSnapShot, this->menuSaveSnapShot});
			this->menuFile->Name = L"menuFile";
			this->menuFile->Size = System::Drawing::Size(36, 20);
			this->menuFile->Text = L"File";
			// 
			// menuFileOpen
			// 
			this->menuFileOpen->Name = L"menuFileOpen";
			this->menuFileOpen->Size = System::Drawing::Size(147, 22);
			this->menuFileOpen->Text = L"Open Pfx File";
			this->menuFileOpen->Click += gcnew System::EventHandler(this, &Form1::menuFileOpen_Click);
			// 
			// menuFileSave
			// 
			this->menuFileSave->Enabled = false;
			this->menuFileSave->Name = L"menuFileSave";
			this->menuFileSave->Size = System::Drawing::Size(147, 22);
			this->menuFileSave->Text = L"Save Pfx File";
			this->menuFileSave->Visible = false;
			this->menuFileSave->Click += gcnew System::EventHandler(this, &Form1::menuFileSave_Click);
			// 
			// menuFileAdd
			// 
			this->menuFileAdd->Enabled = false;
			this->menuFileAdd->Name = L"menuFileAdd";
			this->menuFileAdd->Size = System::Drawing::Size(147, 22);
			this->menuFileAdd->Text = L"Add Pfx File";
			this->menuFileAdd->Visible = false;
			// 
			// menuReset
			// 
			this->menuReset->Enabled = false;
			this->menuReset->Name = L"menuReset";
			this->menuReset->Size = System::Drawing::Size(147, 22);
			this->menuReset->Text = L"Reset";
			this->menuReset->Visible = false;
			this->menuReset->Click += gcnew System::EventHandler(this, &Form1::menuReset_Click);
			// 
			// toolStripSeparator1
			// 
			this->toolStripSeparator1->Name = L"toolStripSeparator1";
			this->toolStripSeparator1->Size = System::Drawing::Size(144, 6);
			// 
			// menuOpenSnapShot
			// 
			this->menuOpenSnapShot->Name = L"menuOpenSnapShot";
			this->menuOpenSnapShot->Size = System::Drawing::Size(147, 22);
			this->menuOpenSnapShot->Text = L"Open Snapshot";
			this->menuOpenSnapShot->Click += gcnew System::EventHandler(this, &Form1::menuOpenSnapShot_Click);
			// 
			// menuSaveSnapShot
			// 
			this->menuSaveSnapShot->Enabled = false;
			this->menuSaveSnapShot->Name = L"menuSaveSnapShot";
			this->menuSaveSnapShot->Size = System::Drawing::Size(147, 22);
			this->menuSaveSnapShot->Text = L"Save Snapshot";
			this->menuSaveSnapShot->Visible = false;
			// 
			// editToolStripMenuItem
			// 
			this->editToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->menuDeleteSelection});
			this->editToolStripMenuItem->Name = L"editToolStripMenuItem";
			this->editToolStripMenuItem->Size = System::Drawing::Size(37, 20);
			this->editToolStripMenuItem->Text = L"Edit";
			// 
			// menuDeleteSelection
			// 
			this->menuDeleteSelection->Name = L"menuDeleteSelection";
			this->menuDeleteSelection->Size = System::Drawing::Size(154, 22);
			this->menuDeleteSelection->Text = L"Delete Selection";
			this->menuDeleteSelection->Click += gcnew System::EventHandler(this, &Form1::menuDeleteSelection_Click);
			// 
			// menuRender
			// 
			this->menuRender->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(9) {this->menuRenderLocalAxis, 
				this->menuRenderAABB, this->menuRenderJoint, this->menuRenderContact, this->menuRenderMeshIslands, this->menuRenderShadow, this->menuRenderSilhouette, 
				this->menuRenderPerformance, this->menuVSync});
			this->menuRender->Name = L"menuRender";
			this->menuRender->Size = System::Drawing::Size(53, 20);
			this->menuRender->Text = L"Render";
			// 
			// menuRenderLocalAxis
			// 
			this->menuRenderLocalAxis->CheckOnClick = true;
			this->menuRenderLocalAxis->Name = L"menuRenderLocalAxis";
			this->menuRenderLocalAxis->Size = System::Drawing::Size(169, 22);
			this->menuRenderLocalAxis->Text = L"Local Axis";
			// 
			// menuRenderAABB
			// 
			this->menuRenderAABB->CheckOnClick = true;
			this->menuRenderAABB->Name = L"menuRenderAABB";
			this->menuRenderAABB->Size = System::Drawing::Size(169, 22);
			this->menuRenderAABB->Text = L"Bounding Volume";
			// 
			// menuRenderJoint
			// 
			this->menuRenderJoint->CheckOnClick = true;
			this->menuRenderJoint->Name = L"menuRenderJoint";
			this->menuRenderJoint->Size = System::Drawing::Size(169, 22);
			this->menuRenderJoint->Text = L"Joint";
			// 
			// menuRenderContact
			// 
			this->menuRenderContact->CheckOnClick = true;
			this->menuRenderContact->Name = L"menuRenderContact";
			this->menuRenderContact->Size = System::Drawing::Size(169, 22);
			this->menuRenderContact->Text = L"Contact";
			// 
			// menuRenderMeshIslands
			// 
			this->menuRenderMeshIslands->CheckOnClick = true;
			this->menuRenderMeshIslands->Name = L"menuRenderMeshIslands";
			this->menuRenderMeshIslands->Size = System::Drawing::Size(169, 22);
			this->menuRenderMeshIslands->Text = L"Large Mesh Islands";
			// 
			// menuRenderShadow
			// 
			this->menuRenderShadow->CheckOnClick = true;
			this->menuRenderShadow->Enabled = false;
			this->menuRenderShadow->Name = L"menuRenderShadow";
			this->menuRenderShadow->Size = System::Drawing::Size(169, 22);
			this->menuRenderShadow->Text = L"Shadow";
			this->menuRenderShadow->Visible = false;
			// 
			// menuRenderSilhouette
			// 
			this->menuRenderSilhouette->CheckOnClick = true;
			this->menuRenderSilhouette->Name = L"menuRenderSilhouette";
			this->menuRenderSilhouette->Size = System::Drawing::Size(169, 22);
			this->menuRenderSilhouette->Text = L"Silhouette";
			// 
			// menuRenderPerformance
			// 
			this->menuRenderPerformance->CheckOnClick = true;
			this->menuRenderPerformance->Name = L"menuRenderPerformance";
			this->menuRenderPerformance->Size = System::Drawing::Size(169, 22);
			this->menuRenderPerformance->Text = L"Performance";
			// 
			// menuVSync
			// 
			this->menuVSync->Checked = true;
			this->menuVSync->CheckOnClick = true;
			this->menuVSync->CheckState = System::Windows::Forms::CheckState::Checked;
			this->menuVSync->Name = L"menuVSync";
			this->menuVSync->Size = System::Drawing::Size(169, 22);
			this->menuVSync->Text = L"VSync";
			this->menuVSync->CheckStateChanged += gcnew System::EventHandler(this, &Form1::menuVSync_CheckStateChanged);
			// 
			// menuConfig
			// 
			this->menuConfig->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->menuConfigWorld});
			this->menuConfig->Name = L"menuConfig";
			this->menuConfig->Size = System::Drawing::Size(50, 20);
			this->menuConfig->Text = L"Config";
			// 
			// menuConfigWorld
			// 
			this->menuConfigWorld->Name = L"menuConfigWorld";
			this->menuConfigWorld->Size = System::Drawing::Size(145, 22);
			this->menuConfigWorld->Text = L"World Property";
			this->menuConfigWorld->Click += gcnew System::EventHandler(this, &Form1::menuConfigWorld_Click);
			// 
			// menuHelp
			// 
			this->menuHelp->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->menuAbout});
			this->menuHelp->Name = L"menuHelp";
			this->menuHelp->Size = System::Drawing::Size(40, 20);
			this->menuHelp->Text = L"Help";
			// 
			// menuAbout
			// 
			this->menuAbout->Name = L"menuAbout";
			this->menuAbout->Size = System::Drawing::Size(100, 22);
			this->menuAbout->Text = L"About";
			this->menuAbout->Click += gcnew System::EventHandler(this, &Form1::menuAbout_Click);
			// 
			// Form1
			// 
			this->AllowDrop = true;
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(592, 545);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->panel1);
			this->Controls->Add(this->menuStrip1);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^  >(resources->GetObject(L"$this.Icon")));
			this->MainMenuStrip = this->menuStrip1;
			this->MinimumSize = System::Drawing::Size(600, 500);
			this->Name = L"Form1";
			this->Text = L"Physics Effects Viewer";
			this->MouseWheel += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::Form1_MouseWheel);
			this->panel1->ResumeLayout(false);
			this->panel1->PerformLayout();
			this->groupBox1->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->EndInit();
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	public: System::Void onUpdate() {
		uint32_t flag = 0;
		if(menuRenderLocalAxis->Checked)	flag |= PfxApp::DrawFlagLocalAxis;
		if(menuRenderAABB->Checked)			flag |= PfxApp::DrawFlagAABB;
		if(menuRenderJoint->Checked)		flag |= PfxApp::DrawFlagJoint;
		if(menuRenderContact->Checked)		flag |= PfxApp::DrawFlagContact;
		if(menuRenderShadow->Checked)		flag |= PfxApp::DrawFlagShadow;
		if(menuRenderSilhouette->Checked)	flag |= PfxApp::DrawFlagSilhouette;
		if(menuRenderPerformance->Checked)	flag |= PfxApp::DrawFlagPerf;
		if(menuRenderMeshIslands->Checked)	flag |= PfxApp::DrawFlagIslandsAABB;
		
		mApp->setDrawFlag(flag);
		mApp->onUpdate();
		mApp->onRender();

		DWORD nextTime = timeGetTime();
		DWORD diffTime = nextTime - mLastTime;

		float sleepTime =  mInterval - (float)diffTime;
		if(sleepTime > 0.0f) {
			Sleep(sleepTime+0.5f);
		}

		mLastTime = timeGetTime();
	}

	private: System::Void pictureBox1_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				 if(mMouseOver) {
					 int diffX = e->X - mMouseX;
					 int diffY = e->Y - mMouseY;
					 mApp->mViewRadX += -0.01f * (float)diffY;
					 mApp->mViewRadY += -0.01f * (float)diffX;
					 mMouseX = e->X;
					 mMouseY = e->Y;
					 if(mApp->mViewRadX < -1.4f)  mApp->mViewRadX = -1.4f;
					 if(mApp->mViewRadX > 0.4f) mApp->mViewRadX = 0.4f;
				 }

				 if(e->Button == System::Windows::Forms::MouseButtons::Left) {
					 int sX = e->X - (pictureBox1->Right-pictureBox1->Left)/2;
					 int sY = (pictureBox1->Bottom-pictureBox1->Top) - e->Y - (pictureBox1->Bottom-pictureBox1->Top)/2;
					 mApp->pickOver(sX,sY);
				 }
			 }
	private: System::Void pictureBox1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				 if(e->Button == System::Windows::Forms::MouseButtons::Right) {
					 mMouseOver = true;
					 mMouseX = e->X;
					 mMouseY = e->Y;
				 }
				 
				 if(e->Button == System::Windows::Forms::MouseButtons::Left) {
					 int sX = e->X - (pictureBox1->Right-pictureBox1->Left)/2;
					 int sY = (pictureBox1->Bottom-pictureBox1->Top) - e->Y - (pictureBox1->Bottom-pictureBox1->Top)/2;

					 PfxApp::ClickMode mode = radioModePick->Checked ? PfxApp::ClickModePick : PfxApp::ClickModeView;
					 mApp->pickClick(sX,sY,mode);

					 textInfo->Clear();
					 string msg;
					 if(mApp->getPickInfo(msg)) {
						 String ^str = gcnew String(msg.c_str());
						 textInfo->AppendText(str);
					 }
				 }
			 }
	private: System::Void pictureBox1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				 if(e->Button == System::Windows::Forms::MouseButtons::Right) {
					 mMouseOver = false;
				 }
				 if(e->Button == System::Windows::Forms::MouseButtons::Left) {
					 int sX = e->X - (pictureBox1->Right-pictureBox1->Left)/2;
					 int sY = (pictureBox1->Bottom-pictureBox1->Top) - e->Y - (pictureBox1->Bottom-pictureBox1->Top)/2;
					 mApp->pickLeave(sX,sY);
				 }
			 }
	private: System::Void pictureBox1_MouseWheel(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 }

	private: System::Void pictureBox1_Resize(System::Object^  sender, System::EventArgs^  e) {
				if(mRender) {
					mRender->setWidth(pictureBox1->Width);
					mRender->setHeight(pictureBox1->Height);
				}
			 }
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
			 }
	private: System::Void menuConfigWorld_Click(System::Object^  sender, System::EventArgs^  e) {
				WorldPropertyDlg^ wpDlg = gcnew WorldPropertyDlg();
				PfxApp::PfxAppWorldProperty wp;
				mApp->getWorldProperty(wp);
				wpDlg->mWorldProperty = &wp;
				wpDlg->ShowDialog();
				mApp->setWorldProperty(wp);
				timer1->Interval = (int)(1000.0f/(float)wp.frameRate);
				mInterval = 1000.0f/(float)wp.frameRate;
				PRINTF("change interval %dmsec\n",(int)(1000.0f/(float)wp.frameRate));
			 }
	private: System::Void menuFileOpen_Click(System::Object^  sender, System::EventArgs^  e) {
				 OpenFileDialog^ dlg = gcnew OpenFileDialog();
				 dlg->Filter = "PhysicsEffects file(*.txt)|*.txt|all files(*.*)|*.*";
				 if(dlg->ShowDialog() != System::Windows::Forms::DialogResult::OK) {
					 return;
				 }

				 char buf[255];
				 sprintf(buf,"%s",dlg->FileName);
				 mApp->importSceneFromPfx(buf);
			 }
	private: System::Void menuFileSave_Click(System::Object^  sender, System::EventArgs^  e) {
			 }
	private: System::Void menuReset_Click(System::Object^  sender, System::EventArgs^  e) {
			 }
	private: System::Void btnSimulate_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
				 mApp->setSimulate(btnSimulate->Checked);
				 if(btnSimulate->Checked) {
					 btnFrame->Enabled = false;
				 }
				 else {
					 btnFrame->Enabled = true;
				 }
			 }
	private: System::Void Form1_MouseWheel(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
				 mApp->mViewRadius *= e->Delta>0.0f?0.9f:1.1f;
				 if(mApp->mViewRadius > 1000.0f) mApp->mViewRadius = 1000.0f;
				 if(mApp->mViewRadius < 0.2f) mApp->mViewRadius = 0.2f;
			 }
private: System::Void radioButton1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void checkLargeMeshIslandsToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void btnFrame_Click(System::Object^  sender, System::EventArgs^  e) {
			mApp->onStartSimulation();
			mApp->onWaitSimulation();
			mApp->onUpdateSimulation();
		 }
private: System::Void menuOpenSnapShot_Click(System::Object^  sender, System::EventArgs^  e) {
				 OpenFileDialog^ dlg = gcnew OpenFileDialog();
				 dlg->Filter = "Snapshot file(*.txt)|*.txt|all files(*.*)|*.*";
				 if(dlg->ShowDialog() != System::Windows::Forms::DialogResult::OK) {
					 return;
				 }

				 char buf[1024];
				 sprintf(buf,"%s",dlg->FileName);
				 mApp->importSceneFromSnapshot(buf);
		 }
private: System::Void menuVSync_CheckStateChanged(System::Object^  sender, System::EventArgs^  e) {
			 mApp->setVSync(menuVSync->Checked);
		 }
private: System::Void menuDeleteSelection_Click(System::Object^  sender, System::EventArgs^  e) {
			 mApp->deleteSelection();
		 }
private: System::Void menuAbout_Click(System::Object^  sender, System::EventArgs^  e) {
			 MessageBox::Show("Copyright (C) 2009 Sony Computer Entertainment Inc.\nAll Rights Reserved.","Physics Effects Viewer",MessageBoxButtons::OK);
		 }

private: System::Void pictureBox1_DragDrop(System::Object^  sender, System::Windows::Forms::DragEventArgs^  e) {
			 if(e->Data->GetDataPresent(DataFormats::FileDrop)) {
				array<System::String^>^ files = static_cast<array<System::String^>^>(e->Data->GetData(DataFormats::FileDrop , false));
				char buf[1024];
				sprintf(buf,"%s",files[0]);
				mApp->importSceneFromPfx(buf);
			 }
		 }
private: System::Void pictureBox1_DragEnter(System::Object^  sender, System::Windows::Forms::DragEventArgs^  e) {
			 e->Effect = DragDropEffects::All;
		 }
};
}

