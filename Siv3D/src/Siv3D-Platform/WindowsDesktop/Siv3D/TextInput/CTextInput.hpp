﻿//-----------------------------------------------
//
//	This file is part of the Siv3D Engine.
//
//	Copyright (c) 2008-2022 Ryo Suzuki
//	Copyright (c) 2016-2022 OpenSiv3D Project
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# pragma once
# include <Siv3D/TextInput/ITextInput.hpp>
# include <Siv3D/Stopwatch.hpp>
# include "WindowsIME.hpp"

namespace s3d
{
	class CTextInput final : public ISiv3DTextInput
	{
	public:

		CTextInput();

		~CTextInput() override;

		void init() override;

		void update() override;

		void pushChar(uint32 ch) override;

		const String& getChars() const override;

		const String& getEditingText() const override;

		void enableIME(bool enabled) override;

		std::pair<int32, int32> getCursorIndex() const override;

		//
		//	Windows
		//

		const Array<String>& getCandidates() const;

		bool process(UINT msg, WPARAM wParam, LPARAM* lParam);

		void sendEditingText(const String& text, int32 cursorPos, int32 targetLength);

		void sendCandidates(const Array<String>& list);

		void sendInputText(const String& text);

	private:

		HWND m_hWnd = nullptr;

		HIMC m_hImc = nullptr;

		Stopwatch m_deletePress;

		////////
		//
		std::mutex m_mutex;

		String m_internalEditingText;

		int32 m_internalCursorPos = 0;

		int32 m_internalTargetLength = 0;

		Array<String> m_internalCandidates;

		String m_internalChars;
		//
		////////

		String m_editingText;

		int32 m_cursorPos = 0;

		int32 m_targetLength = 0;

		Array<String> m_candidates;

		String m_chars;


		bool m_enabled = true;

		mutable bool m_editingTextHandled = true;

		std::unique_ptr<SDL_VideoData> m_pVideo;
	};
}
